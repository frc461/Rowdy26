package frc.robot.subsystems.localizer;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;

public class Localizer {

    private final SwerveDrivePoseEstimator poseEstimator;

    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public final PhotonPoseEstimator frEstimator;
    public final PhotonPoseEstimator flEstimator;
    public final PhotonPoseEstimator brEstimator;
    public final PhotonPoseEstimator blEstimator;

    private final PhotonCamera frCam;
    private final PhotonCamera flCam;
    private final PhotonCamera brCam;
    private final PhotonCamera blCam;

    //TODO: ADD CONSTANTS
    private static final Transform3d kRobotTofrCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
    private static final Transform3d kRobotToflCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
    private static final Transform3d kRobotTobrCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
    private static final Transform3d kRobotToblCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));

    public Localizer() {
        //TODO: ADD CONSTANTS
        poseEstimator = new SwerveDrivePoseEstimator(null, null, null, null);

        frCam = new PhotonCamera("FR");
        flCam = new PhotonCamera("FL");
        brCam = new PhotonCamera("BR");
        blCam = new PhotonCamera("BL");

        frEstimator = new PhotonPoseEstimator(kTagLayout, kRobotTofrCam);
        flEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToflCam);
        brEstimator = new PhotonPoseEstimator(kTagLayout, kRobotTobrCam);
        blEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToblCam);
    }

    //TODO: Standard Deviation Stuff
    private EstimatedRobotPose cameraEstimate(PhotonCamera camera, PhotonPoseEstimator photonEstimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            // updateEstimationStdDevs(visionEst, result.getTargets());

            // if (Robot.isSimulation()) {
            //     visionEst.ifPresentOrElse(
            //             est ->
            //                     getSimDebugField()
            //                             .getObject("VisionEstimation")
            //                             .setPose(est.estimatedPose.toPose2d()),
            //             () -> {
            //                 getSimDebugField().getObject("VisionEstimation").setPoses();
            //             });
            // }

            // visionEst.ifPresent(
            //         est -> {
            //             // Change our trust in the measurement based on the tags we can see
            //             var estStdDevs = getEstimationStdDevs();

            //             estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            // });
        }
        if(visionEst.isPresent()) {
            return (visionEst.get());
        }
        else {
            return null;
        }
    }        

    public void updatePhotonEstimation() {
        var frEst = cameraEstimate(frCam, frEstimator);
        var flEst = cameraEstimate(flCam, flEstimator);
        var brEst = cameraEstimate(brCam, brEstimator);
        var blEst = cameraEstimate(blCam, blEstimator);

        poseEstimator.addVisionMeasurement(frEst.estimatedPose.toPose2d(), frEst.timestampSeconds);
        poseEstimator.addVisionMeasurement(flEst.estimatedPose.toPose2d(), flEst.timestampSeconds);
        poseEstimator.addVisionMeasurement(brEst.estimatedPose.toPose2d(), brEst.timestampSeconds);
        poseEstimator.addVisionMeasurement(blEst.estimatedPose.toPose2d(), blEst.timestampSeconds);

        //TODO: Add Constants
        poseEstimator.update(null, null);

    }


    



}
