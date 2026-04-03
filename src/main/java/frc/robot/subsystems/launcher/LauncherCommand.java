package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.launcher.ShooterSolver.ShotResult;
import frc.robot.subsystems.drivetrain.Swerve;

public class LauncherCommand extends Command {
    private final Swerve drivetrain;
    private final Launcher launcher;
    
    public LauncherCommand(Swerve drivetrain, Launcher launcher) {
        this.drivetrain = drivetrain;
        this.launcher = launcher;
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
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stopFlyWheels();
    }
}
