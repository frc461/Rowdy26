package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Transform2d;
<<<<<<< HEAD
import java.util.List;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;


=======
import org.photonvision.targeting.TargetCorner;

import java.lang.ProcessBuilder.Redirect;
import java.util.List;
import java.util.concurrent.ForkJoinTask;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
>>>>>>> f51328517b916fc621ba03c74f510fd447d8cf3f

public final class Vision {
    private final PhotonCamera camera = new PhotonCamera("camera1");
    private List<PhotonPipelineResult> lastResults;
<<<<<<< HEAD

    public List<PhotonPipelineResult> getVision() {
        lastResults = camera.getAllUnreadResults();
        return lastResults;
    }
    
=======
    

    public List<PhotonPipelineResult> getVision() {
        lastResults = camera.getAllUnreadResults();
        return lastResults;
    }
    
>>>>>>> f51328517b916fc621ba03c74f510fd447d8cf3f
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
<<<<<<< HEAD

     List<PhotonTrackedTarget> targets = result.getTargets();{
            for (PhotonTrackedTarget target : targets) {
                double yaw = target.getYaw();
                double pitch = target.getPitch();
                double area = target.getArea();
                double skew = target.getSkew();
                //Transform2d pose = target.getCameraToTarget();
                //List<TargetCorner> corners = target.getCorners();
                // get info from target
                //int targetID = target.getFiducicialID();
                double poseAmbiguity = target.getPoseAmbiguity();
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

                // Add additional processing as needed
        }       
    }
    }
    }
    
}
=======

     List<PhotonTrackedTarget> targets = result.getTargets();{
            for (PhotonTrackedTarget target : targets) {
                double yaw = target.getYaw();
                double pitch = target.getPitch();
                double area = target.getArea();
                double skew = target.getSkew();
                //Transform2d pose = target.getCameraToTarget();
                //List<TargetCorner> corners = target.getCorners();
                // get info from target
                //int targetID = target.getFiducicialID();
                double poseAmbiguity = target.getPoseAmbiguity();
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

                // Add additional processing as needed
        }       
    } 
    }
    }


    public void BWCamera( ) {
        //RED

        //GRAY 

    } 
}

    



        
>>>>>>> f51328517b916fc621ba03c74f510fd447d8cf3f

