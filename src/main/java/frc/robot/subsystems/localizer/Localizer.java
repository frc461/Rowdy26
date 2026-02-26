package frc.robot.subsystems.localizer;

import java.util.List;
import java.util.Optional;


import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.vision.Vision.BW;
import frc.robot.util.vision.Vision.BW.BWCamera;
import frc.robot.util.vision.Vision;

public class Localizer extends SubsystemBase {

    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final Vision vision;
    private final SwerveDrivePoseEstimator poseEstimator;
    
    private final Swerve swerve;

    public Field2d fieldOdom;

    private Pose2d currentPose;


    public Localizer(Swerve swerve) {

        currentPose = new Pose2d();
        fieldOdom = new Field2d();

        this.swerve = swerve;

        poseEstimator = new SwerveDrivePoseEstimator(
            this.swerve.getKinematics(), 
            this.swerve.getState().RawHeading,
            this.swerve.getState().ModulePositions,
            this.swerve.getState().Pose,
            Constants.VisionConstants.ODOM_STD_DEV,
            Constants.VisionConstants.kMULTI_TAG_STD_DEVS
        );
       
        vision = new Vision((pose, timestamp, stdDevs) -> {
            poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
        });

        //TEMP, unsure if correct.
        SmartDashboard.putData("Field", fieldOdom);

    }

    public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d newPose) {
        poseEstimator.resetPosition(gyroAngle, modulePositions, newPose);
    }

    public void updateOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(gyroAngle, modulePositions);
    }

    public void periodic() {
        currentPose = poseEstimator.getEstimatedPosition();
        vision.getEstimatedGlobalPoses(currentPose);

        fieldOdom.setRobotPose(currentPose);
    }
    

 }

