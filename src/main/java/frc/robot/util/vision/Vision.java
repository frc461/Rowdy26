package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;

import java.lang.ProcessBuilder.Redirect;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ForkJoinTask;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.localizer.Localizer.EstimateConsumer;
import frc.robot.util.vision.Vision.BW.BWCamera;
//import frc.robot.util.EstimatedRobotPose;
import frc.robot.util.FieldUtil;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public final class Vision extends SubsystemBase{

    private List<PhotonPipelineResult> latestResults;
    
    private Matrix <N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    public static PhotonPipelineResult latestResultCameraFR = new PhotonPipelineResult();
    public static PhotonPipelineResult latestResultCameraFL = new PhotonPipelineResult();
    public static PhotonPipelineResult latestResultCameraBR = new PhotonPipelineResult();
    public static PhotonPipelineResult latestResultCameraBL = new PhotonPipelineResult();

    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final Map<BWCamera, PhotonPoseEstimator> estimators = new EnumMap<>(BWCamera.class);    

    //TODO: PUBLISH VALUES TO NETWORK TABLES
    public Vision(EstimateConsumer estC) {
        for(BWCamera cam : BW.BWCamera.values()) {
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(kTagLayout, cam.robotToCameraOffset);

            estimators.put(cam, estimator);
        }

        estConsumer = estC;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> visionEst, List<PhotonTrackedTarget> targets) {
        
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
    
    public List<EstimatedRobotPose> getEstimatedGlobalPoses(Pose2d referencePose) {
        List<EstimatedRobotPose> allEstimates = new ArrayList<>();
        for(BWCamera cam : BWCamera.values()) {
            PhotonPoseEstimator estimator = estimators.get(cam);

            if(estimator == null) continue;

            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for (var result : cam.camera.getAllUnreadResults()) {
                visionEst = estimator.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = estimator.estimateLowestAmbiguityPose(result);
                }
                updateEstimationStdDevs(visionEst, result.getTargets());

                // if (Robot.isSimulation()) {
                //     visionEst.ifPresentOrElse(
                //         est ->
                //                 getSimDebugField()
                //                         .getObject("VisionEstimation")
                //                         .setPose(est.estimatedPose.toPose2d()),
                //         () -> {
                //             getSimDebugField().getObject("VisionEstimation").setPoses();
                //         });
            }
            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();

                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
            }
            return allEstimates;
        }






    
    
    public boolean hasTargets() {
        if (latestResults == null || latestResults.isEmpty()) {
            return false;
        }
        for (PhotonPipelineResult r : latestResults) {
            if (r.hasTargets()) return true;
        }
        return false;
    }
    
    public List<PhotonPipelineResult> getCameraToTarget() {
        return latestResults;
    }
    
    public void processTargets(PhotonPipelineResult result){
        getCameraToTarget();{
            
        }  
     {

     List<PhotonTrackedTarget> targets = result.getTargets();{
            for (PhotonTrackedTarget target : targets) {
                double yaw = target.getYaw();
                double pitch = target.getPitch();
                double area = target.getArea();
                double skew = target.getSkew();
                //Transform2d pose = target.getCameraToTarget();
                //List<TargetCorner> corners = target.getCorners();
                // get info from target
                int targetId = target.getFiducialId();
                double poseAmbiguity = target.getPoseAmbiguity();
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

                // Add additional processing as needed
            }       
        } 
    }
    }

    //CAMERA ENUM
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
            ),

            CAMERA_BL (
                new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.CAMERA_BL_NAME),
                new Transform3d(
                    Constants.VisionConstants.CAMERA_BL_FORWARD,
                    Constants.VisionConstants.CAMERA_BL_LEFT,
                    Constants.VisionConstants.CAMERA_BL_UP,
                    new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_BL_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_BL_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_BL_YAW)
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



    public static PhotonPipelineResult getLatestResult(BWCamera camera){
        return switch (camera){
            case CAMERA_FR -> latestResultCameraFR;
            case CAMERA_FL -> latestResultCameraFL;
            case CAMERA_BR -> latestResultCameraBR;
            case CAMERA_BL -> latestResultCameraBL;
        };
    }
    public static boolean hasTargets(BWCamera camera) {
        return switch (camera) {
            case CAMERA_FR -> latestResultCameraFR.hasTargets();
            case CAMERA_FL -> latestResultCameraFL.hasTargets();
            case CAMERA_BR -> latestResultCameraBR.hasTargets();
            case CAMERA_BL -> latestResultCameraBL.hasTargets();
        };
        
    }
}