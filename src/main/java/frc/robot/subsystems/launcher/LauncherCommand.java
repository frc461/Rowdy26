package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.launcher.ShooterSolver.ShotResult;

public class LauncherCommand extends Command {
    private final Launcher launcher;
    private final Swerve drivetrain;
    
    public LauncherCommand(Launcher launcher, Swerve drivetrain) {
        this.launcher = launcher;
        this.drivetrain = drivetrain;
        addRequirements(launcher);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        ShotResult solution = ShooterSolver.solve(currentPose);

        if (solution.found) {
            launcher.setFlywheelVelocity(-1.0 * solution.rpm);
            launcher.setHoodPosition(Constants.LauncherConstants.AUTO_AIM_HOOD_ANGLE);
            SmartDashboard.putBoolean("Shooter/Solution Found", true);
        } else {
            SmartDashboard.putBoolean("Shooter/Solution Found", false);
        }
        
        SmartDashboard.putNumber("Shooter/Target RPM", solution.rpm);
        SmartDashboard.putNumber("Shooter/Target Hood Angle", Constants.LauncherConstants.AUTO_AIM_HOOD_ANGLE);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop flywheel when ended, similar to AimAtHubCommand's commented out ending
        launcher.stopFlyWheels();
    }
}
