package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import frc.robot.constants.variants.DefaultConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.util.vision.Vision.BW.BWCamera;

public final class Vision extends SubsystemBase {

    /**
     * Functional interface allowing the Vision subsystem to pass data 
     * directly into the CTRE drivetrain's pose estimator.
     */
    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    // List to hold the raw camera poses for the Elastic dashboard "ghost" robots
    private final List<Pose2d> currentVisionPoses = new ArrayList<>();

    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final Map<BWCamera, PhotonPoseEstimator> estimators = new EnumMap<>(BWCamera.class);   

    public Vision(EstimateConsumer estC) {
        
        // Initialize all estimators based on the enum
        for(BWCamera cam : BW.BWCamera.values()) {
            
            // 2026 STANDARD: Constructor only accepts the layout and the camera's physical offset
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                kTagLayout, 
                cam.getRobotToCameraOffset()
            );
            estimators.put(cam, estimator);
        }

        estConsumer = estC;
    }

    /**
     * Dynamically adjusts how much we trust the vision measurement based on 
     * the number of tags visible and how far away they are.
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator photonEstimator) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = DefaultConstants.VisionConstants.kSINGLE_TAG_STD_DEVS;
            return;
        } 

        var estStdDevs = DefaultConstants.VisionConstants.kSINGLE_TAG_STD_DEVS;
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
            curStdDevs = DefaultConstants.VisionConstants.kSINGLE_TAG_STD_DEVS;
        } else {
            avgDist /= numTags;
            // Decrease std devs (increase trust) if multiple targets are visible
            if (numTags > 1) estStdDevs = DefaultConstants.VisionConstants.kMULTI_TAG_STD_DEVS;
            // Increase std devs (decrease trust) if a single tag is very far away
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
     * Called continuously from the Localizer's periodic loop.
     * Reads all cameras, stores the raw poses for the dashboard, and feeds the filtered data to CTRE.
     * @param referencePose The current robot pose, used to resolve 1-tag ambiguity.
     */
    public void updateVisionPoses(Pose2d referencePose) {
        // Clear the ghost list at the start of every 20ms loop
        currentVisionPoses.clear();

        for(BWCamera cam : BWCamera.values()) {
            PhotonPoseEstimator estimator = estimators.get(cam);
            if(estimator == null) continue;
            
            // Safely iterate through all new frames to ensure we don't miss data at high speeds
            for (var result : cam.getCamera().getAllUnreadResults()) {
                
                // 2026 STANDARD: Attempt to calculate a highly accurate multi-tag pose first
                Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
                
                // Convert our 2D swerve odometry into a 3D pose for PhotonVision math
                if (visionEst.isEmpty()) {
                    Pose3d referencePose3d = new Pose3d(referencePose);
                    visionEst = estimator.estimateClosestToReferencePose(result, referencePose3d);
                }

                // FIX: Use a standard 'if' statement instead of a lambda to avoid the "effectively final" error
                if (visionEst.isPresent()) {
                    EstimatedRobotPose est = visionEst.get();
                    Pose2d rawVisionPose = est.estimatedPose.toPose2d();
                    
                    // 1. ALWAYS add the raw pose to our list for the dashboard ghost robots
                    // This lets the drivers verify cameras are working during auto
                    currentVisionPoses.add(rawVisionPose);

                    // 2. ONLY feed the data to the Swerve drive if we are NOT in Autonomous
                    if (!DriverStation.isAutonomous()) {
                        // Calculate trust based on distance and number of tags
                        updateEstimationStdDevs(visionEst, est.targetsUsed, estimator);
                        Matrix<N3, N1> stdDevs = getEstimationStdDevs();
                        
                        // Pass the data directly into the CTRE drivetrain
                        estConsumer.accept(rawVisionPose, est.timestampSeconds, stdDevs);
                    }
                }
            }
        }
    }

    /**
     * Returns the list of raw camera poses from the current loop for the Field2d widget.
     */
    public List<Pose2d> getVisionRobotPoses() {
        return currentVisionPoses;
    }

    // --- CAMERA ENUM CONFIGURATION ---
    public static final class BW {
        public enum BWCamera {
            CAMERA_FR (
                new PhotonCamera(DefaultConstants.VisionConstants.CAMERA_FR_NAME),
                new Transform3d(
                    DefaultConstants.VisionConstants.CAMERA_FR_FORWARD,
                    DefaultConstants.VisionConstants.CAMERA_FR_LEFT,
                    DefaultConstants.VisionConstants.CAMERA_FR_UP,
                    new Rotation3d(
                        Units.degreesToRadians(DefaultConstants.VisionConstants.CAMERA_FR_PITCH),
                        Units.degreesToRadians(DefaultConstants.VisionConstants.CAMERA_FR_ROLL),
                        Units.degreesToRadians(DefaultConstants.VisionConstants.CAMERA_FR_YAW)
                    )
                )
            ),
            
            CAMERA_FL (
                new PhotonCamera(DefaultConstants.VisionConstants.CAMERA_FL_NAME),
                new Transform3d(
                    DefaultConstants.VisionConstants.CAMERA_FL_FORWARD,
                    DefaultConstants.VisionConstants.CAMERA_FL_LEFT,
                    DefaultConstants.VisionConstants.CAMERA_FL_UP,
                    new Rotation3d(
                        Units.degreesToRadians(DefaultConstants.VisionConstants.CAMERA_FL_PITCH),
                        Units.degreesToRadians(DefaultConstants.VisionConstants.CAMERA_FL_ROLL),
                        Units.degreesToRadians(DefaultConstants.VisionConstants.CAMERA_FL_YAW)
                    )
                )
            ),

            CAMERA_BR (
                new PhotonCamera(DefaultConstants.VisionConstants.CAMERA_BR_NAME),
                new Transform3d(
                    DefaultConstants.VisionConstants.CAMERA_BR_FORWARD,
                    DefaultConstants.VisionConstants.CAMERA_BR_LEFT,
                    DefaultConstants.VisionConstants.CAMERA_BR_UP,
                    new Rotation3d(
                        Units.degreesToRadians(DefaultConstants.VisionConstants.CAMERA_BR_PITCH),
                        Units.degreesToRadians(DefaultConstants.VisionConstants.CAMERA_BR_ROLL),
                        Units.degreesToRadians(DefaultConstants.VisionConstants.CAMERA_BR_YAW)
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

    /**
     * Helper method to check if a specific camera currently sees a target.
     */
    public boolean hasTargets(BWCamera camera) {
        var result = camera.getCamera().getLatestResult();
        return result.hasTargets();
    }
}