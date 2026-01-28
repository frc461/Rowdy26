package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d; //rotation2d
import org.photonvision.PhotonCamera; 
import edu.wpi.first.math.util.Units;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;



public final class PhotonUtil {
    public static void updateResults(Rotation2d heading){ 
        BW.updateResults(heading);
    }
}
