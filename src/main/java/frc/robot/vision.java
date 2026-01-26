package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import org.photonvision.targeting.TargetCorner;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public final class vision {
    private final PhotonCamera camera = new PhotonCamera("camera1");
    private List<PhotonPipelineResult> lastResults;

    public List<PhotonPipelineResult> getVision() {
        lastResults = camera.getAllUnreadResults();
        return lastResults;
    }
    
    public boolean hasTargets() {
        if (lastResults == null || lastResults.isEmpty()) {
            return false;
        }
        for (PhotonPipelineResult r : lastResults) {
            if (r.hasTargets()) return true;
        }
        return false;
    }
    public static void processTargets(PhotonPipelineResult result){
        static getCameraToTarget(PhotonPipelineResult result){
         }  
     {

     List<PhotonTrackedTarget> targets = result.getTargets();{
            for (PhotonTrackedTarget target : targets) {
                double yaw = target.getYaw();
                double pitch = target.getPitch();
                double area = target.getArea();
                double skew = target.getSkew();
                Transform2d pose = target.getCameraToTarget();
                List<TargetCorner> corners = target.getCorners();
                // Add additional processing as needed

    

        }       
    } 
    
    
     }
    }
}
        

