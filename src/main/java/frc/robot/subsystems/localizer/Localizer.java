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
import edu.wpi.first.math.Matrix;
import frc.robot.Robot;
import frc.robot.util.vision.Vision.BW;
import frc.robot.util.vision.Vision.BW.BWCamera;
import frc.robot.util.vision.Vision;

public class Localizer {

    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final Vision vision;
    private final SwerveDrivePoseEstimator poseEstimator;
    
    private final NetworkTable odoTable;
    private final StructPublisher<Pose2d> posePublisher;

    public Localizer(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPose) {
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPose);
        vision = new Vision((pose, timestamp, stdDevs) -> {
            poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
        });

        //TEMP, unsure if correct.
        odoTable = NetworkTableInstance.getDefault().getTable("Odometry");
        posePublisher = odoTable.getStructTopic("RobotPose", Pose2d.struct).publish();

    }

    public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d newPose) {
        poseEstimator.resetPosition(gyroAngle, modulePositions, newPose);
    }

    public void updateOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(gyroAngle, modulePositions);
    }

    public void periodic() {
        Pose2d currentPose = poseEstimator.getEstimatedPosition();

        vision.getEstimatedGlobalPoses(currentPose); 
    }
    

 }

