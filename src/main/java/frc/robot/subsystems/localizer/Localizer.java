package frc.robot.subsystems.localizer;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Localizer {
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    PhotonCamera frCam = new PhotonCamera("FR");
    PhotonCamera flCam = new PhotonCamera("FL");
    PhotonCamera brCam = new PhotonCamera("BR");
    PhotonCamera blCam = new PhotonCamera("BL");


//TODO: Reference Constants
    public static final Transform3d kRobotTofrCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
    public static final Transform3d kRobotToflCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
    public static final Transform3d kRobotTobrCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
    public static final Transform3d kRobotToblCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
    
    private final PhotonPoseEstimator frEstimator = new PhotonPoseEstimator(kTagLayout, kRobotTofrCam);
    private final PhotonPoseEstimator flEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToflCam);
    private final PhotonPoseEstimator brEstimator = new PhotonPoseEstimator(kTagLayout, kRobotTobrCam);
    private final PhotonPoseEstimator blEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToblCam);
}
