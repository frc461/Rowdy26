package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.ShooterSolver;
import frc.robot.subsystems.launcher.ShooterSolver.ShotResult;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveCommand extends Swerve implements Subsystem{

    // 1. Instantiate the Field2d object
    private final Field2d m_field = new Field2d();

    public boolean isAutoAiming = false;


    public SwerveCommand (SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
       
        // 2. Publish the Field2d object to NetworkTables
        SmartDashboard.putData("Field2d Pose", m_field);


        PPHolonomicDriveController.setRotationTargetOverride(() -> {
            if (isAutoAiming) {
                // Run the math
                Pose2d currentPose = this.getState().Pose;
                Translation2d currentSpeeds = new Translation2d(
                    this.getState().Speeds.vxMetersPerSecond,
                    this.getState().Speeds.vyMetersPerSecond
                ).rotateBy(currentPose.getRotation());

                ShotResult solution = ShooterSolver.solve(currentPose);

                // Hijack PathPlanner's rotation and force it to face the target!
                return Optional.of(Rotation2d.fromDegrees(solution.headingDegrees));
            } else {
                // Return empty to let PathPlanner rotate normally based on the drawn path
                return Optional.empty();
            }
        });
    }

    // ... other constructors ...
    public void setAutoAim(boolean enable) {
            this.isAutoAiming = enable;
        }

    @Override
    public void periodic() {
        // 3. Extract the high-frequency pose and update the field widget
        m_field.setRobotPose(this.getState().Pose);
    }
}



