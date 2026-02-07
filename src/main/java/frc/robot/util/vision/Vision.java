package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.util.Units;

import java.lang.ProcessBuilder.Redirect;
import java.util.List;
import java.util.concurrent.ForkJoinTask;

import org.photonvision.PhotonCamera;
import frc.robot.constants.Constants;
import frc.robot.util.vision.Vision.BW.BWCamera;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public final class Vision {
    // private final PhotonCamera REDcamera = new PhotonCamera("RED");
    // private final PhotonCamera GRAYcamera = new PhotonCamera("GRAY");

    private List<PhotonPipelineResult> lastResults;

    public static PhotonPipelineResult lastResultRed = new PhotonPipelineResult();
    public static PhotonPipelineResult lastResultGray = new PhotonPipelineResult();

    

    // public List<PhotonPipelineResult> getVision() {
    //     lastResults = REDcamera.getAllUnreadResults();
    //     return lastResults;
    // }
    
    public boolean hasTargets() {
        if (lastResults == null || lastResults.isEmpty()) {
            return false;
        }
        for (PhotonPipelineResult r : lastResults) {
            if (r.hasTargets()) return true;
        }
        return false;
    }
    
    
    public List<PhotonPipelineResult> getCameraToTarget() {
        return lastResults;
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


    public static final class BW {
        public enum BWCamera {
            CAMERA_RED (
                new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.CAMERA_RED_NAME),
                new Transform3d(
                    Constants.VisionConstants.CAMERA_RED_FORWARD,
                    Constants.VisionConstants.CAMERA_RED_LEFT,
                    Constants.VisionConstants.CAMERA_RED_UP,
                    new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_RED_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_RED_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_RED_YAW)
                    )
                )
            ),
            
            CAMERA_GRAY (
                new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.CAMERA_GRAY_NAME),
                new Transform3d(
                    Constants.VisionConstants.CAMERA_GRAY_FORWARD,
                    Constants.VisionConstants.CAMERA_GRAY_LEFT,
                    Constants.VisionConstants.CAMERA_GRAY_UP,
                    new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_GRAY_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_GRAY_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_GRAY_YAW)
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
    public static PhotonPipelineResult latestResultCameraRed = new PhotonPipelineResult();
    public static PhotonPipelineResult latestResultCameraGray = new PhotonPipelineResult();

    public static PhotonPipelineResult getLatestResult(BWCamera camera){
        return switch (camera){
            case CAMERA_RED -> latestResultCameraRed;
            case CAMERA_GRAY -> latestResultCameraGray;
        };
    }
    public static boolean hasTargets(BWCamera camera) {
        return switch (camera) {
            case CAMERA_RED -> latestResultCameraRed.hasTargets();
            case CAMERA_GRAY -> latestResultCameraRed.hasTargets();
        };
        
    }


    //single tag pose
    
    
}