package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.launcher.ShooterSolver;
import frc.robot.subsystems.launcher.ShooterSolver.ShotResult;

public class LauncherCommand extends Command {
    private final Launcher launcher;
    private final Spindexer spindexer;
    private final Swerve drivetrain; 
    private final Intake intake; 

    private final boolean isAutoAim;
    private final double presetRpm;
    private final double presetHoodAngle;

    public LauncherCommand(Launcher launcher, Spindexer spindexer, Swerve drivetrain, Intake intake) {
        this.launcher = launcher;
        this.spindexer = spindexer;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.isAutoAim = true;
        this.presetRpm = 0;
        this.presetHoodAngle = 0;
        
        addRequirements(launcher, spindexer); 
    }

    public LauncherCommand(Launcher launcher, Spindexer spindexer, Intake intake, double rpm, double hoodAngle) {
        this.launcher = launcher;
        this.spindexer = spindexer;
        this.drivetrain = null;
        this.intake = intake;
        this.isAutoAim = false;
        this.presetRpm = rpm;
        this.presetHoodAngle = hoodAngle;
        
        addRequirements(launcher, spindexer);
    }

    @Override
    public void execute() {
        double targetRpm;
        double targetHood;
        boolean readyToFire;

        if (isAutoAim) {
            // Only pass the Pose! Speed is ignored.
            ShotResult solution = ShooterSolver.solve(drivetrain.getState().Pose);
            
            targetRpm = -solution.rpm;
            targetHood = solution.hoodAngle; 
            
            // boolean isSpunUp = Math.abs(launcher.getFlywheelVelocity() - targetRpm) < 50.0;
            boolean isSpunUp = launcher.getFlywheelVelocity() < targetRpm;
            readyToFire = isSpunUp && drivetrain.isAimLockedOn() && solution.found;
            
        } else {
            targetRpm = presetRpm;
            targetHood = presetHoodAngle;

            // boolean isSpunUp = Math.abs(launcher.getFlywheelVelocity() - targetRpm) < 50.0;
            boolean isSpunUp = launcher.getFlywheelVelocity() < targetRpm;
            readyToFire = isSpunUp;
        }

        launcher.setFlywheelVelocity(targetRpm);
        launcher.setHoodPosition(targetHood);

        launcher.runFlyWheel();
        launcher.runHood();

        if (readyToFire) {
            spindexer.setVoltage(16); 
            intake.setIntakeVoltage(-16); 
        } else {
            spindexer.setVoltage(0);  
            intake.setIntakeVoltage(0); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stopFlyWheels();
        launcher.setHoodPosition(0.0);
        launcher.runHood();
        spindexer.setVoltage(0);
        intake.setIntakeVoltage(0);
    }
}