package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import frc.robot.constants.Constants;
import frc.robot.util.vision.Vision.BW.BWCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public final class Vision extends SubsystemBase {

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    private final List<Pose2d> currentVisionPoses = new java.util.ArrayList<>();

    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final Map<BWCamera, PhotonPoseEstimator> estimators = new EnumMap<>(BWCamera.class);   
    private final NetworkTable visionTable;

    public Vision(EstimateConsumer estC) {
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");

        // Initialize all estimators based on the enum
        for(BWCamera cam : BW.BWCamera.values()) {
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                kTagLayout, 
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cam.getCamera(),
                cam.getRobotToCameraOffset()
            );
            estimators.put(cam, estimator);
        }

        estConsumer = estC;
    }

    public void updateVisionPoses(Pose2d referencePose) {
        // Clear the list at the start of every loop
        currentVisionPoses.clear();

        for(BWCamera cam : BWCamera.values()) {
            PhotonPoseEstimator estimator = estimators.get(cam);
            if(estimator == null) continue;

            estimator.setReferencePose(referencePose);
            Optional<EstimatedRobotPose> visionEst = estimator.update();

            visionEst.ifPresent(est -> {
                Pose2d rawVisionPose = est.estimatedPose.toPose2d();
                
                // 1. Add the raw pose to our list for the dashboard
                currentVisionPoses.add(rawVisionPose);

                // 2. Feed the data to CTRE (as usual)
                updateEstimationStdDevs(visionEst, est.targetsUsed, estimator);
                Matrix<N3, N1> stdDevs = getEstimationStdDevs();
                estConsumer.accept(rawVisionPose, est.timestampSeconds, stdDevs);
            });
        }
    }

    /**
     * Returns the list of raw vision poses from the current loop.
     */
    public List<Pose2d> getVisionRobotPoses() {
        return currentVisionPoses;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator photonEstimator) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = Constants.VisionConstants.kSINGLE_TAG_STD_DEVS;
            return;
        } 

        var estStdDevs = Constants.VisionConstants.kSINGLE_TAG_STD_DEVS;
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            curStdDevs = Constants.VisionConstants.kSINGLE_TAG_STD_DEVS;
        } else {
            avgDist /= numTags;
            if (numTags > 1) estStdDevs = Constants.VisionConstants.kMULTI_TAG_STD_DEVS;
            if (numTags == 1 && avgDist > 4) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            }
            curStdDevs = estStdDevs;
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
    
    /**
     * Called continuously to read from all cameras and feed estimates to the consumer.
     */
    public void updateVisionPoses(Pose2d referencePose) {
        for(BWCamera cam : BWCamera.values()) {
            PhotonPoseEstimator estimator = estimators.get(cam);
            if(estimator == null) continue;

            // Setting the reference pose helps resolve ambiguity when only one tag is visible
            estimator.setReferencePose(referencePose);
            
            Optional<EstimatedRobotPose> visionEst = estimator.update();

            visionEst.ifPresent(est -> {
                // Calculate trust based on distance and number of tags
                updateEstimationStdDevs(visionEst, est.targetsUsed, estimator);
                Matrix<N3, N1> stdDevs = getEstimationStdDevs();
                
                // Pass the data to the CTRE drivetrain
                estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs);
            });
        }
    }

    // CAMERA ENUM
    public static final class BW {
        public enum BWCamera {
            CAMERA_FR (
                new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.CAMERA_FR_NAME),
                new Transform3d(
                    Constants.VisionConstants.CAMERA_FR_FORWARD,
                    Constants.VisionConstants.CAMERA_FR_LEFT,
                    Constants.VisionConstants.CAMERA_FR_UP,
                    new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FR_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FR_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FR_YAW)
                    )
                )
            ),
            
            CAMERA_FL (
                new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.CAMERA_FL_NAME),
                new Transform3d(
                    Constants.VisionConstants.CAMERA_FL_FORWARD,
                    Constants.VisionConstants.CAMERA_FL_LEFT,
                    Constants.VisionConstants.CAMERA_FL_UP,
                    new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FL_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FL_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FL_YAW)
                    )
                )
            ),

            CAMERA_BR (
                new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.CAMERA_BR_NAME),
                new Transform3d(
                    Constants.VisionConstants.CAMERA_BR_FORWARD,
                    Constants.VisionConstants.CAMERA_BR_LEFT,
                    Constants.VisionConstants.CAMERA_BR_UP,
                    new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_BR_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_BR_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_BR_YAW)
                    )
                )
            );

            final PhotonCamera camera; 
            final Transform3d robotToCameraOffset;
            
            BWCamera(PhotonCamera camera, Transform3d robotToCameraOffset) {
                this.camera = camera; 
                this.robotToCameraOffset = robotToCameraOffset;
            }

            public PhotonCamera getCamera(){
                return camera;
            }
            
            public Transform3d getRobotToCameraOffset(){
                return robotToCameraOffset;
            }
        }
    } 
}