package frc.robot;

import frc.robot.constants.Constants;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherCommand;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.intake.Intake;

public class AutoCommand extends SubsystemBase {

    private Launcher launcher;
    private Spindexer spindexer;
    private Intake intake;
    private Swerve drivetrain;

    public AutoCommand(Launcher m_Launcher, Spindexer m_Spindexer, Intake m_Intake, Swerve m_Drivetrain)
    {
        launcher = m_Launcher;
        spindexer = m_Spindexer;
        intake = m_Intake;
        drivetrain = m_Drivetrain;
    }

    public Command AutoHumanPlayerShoot() {
        return runOnce(() -> {
            launcher.setFlywheelVelocity(Constants.LauncherConstants.TEMP_AUTO_RPM);
            launcher.setHoodPosition(Constants.LauncherConstants.TEMP_AUTO_START_HOOD_ANGLE);
            launcher.runFlyWheel();
            launcher.runHood();
            spindexer.setVoltage(16);
            intake.setIntakeVoltage(-16);
            }
        );
    }

    public Command AutoTrenchShoot() {
        return runOnce(() -> {
            launcher.setFlywheelVelocity(Constants.LauncherConstants.TRENCH_AUTO_RPM);
            launcher.setHoodPosition(Constants.LauncherConstants.TRENCH_AUTO_START_HOOD_ANGLE);
            launcher.runFlyWheel();
            launcher.runHood();
            spindexer.setVoltage(16);
            intake.setIntakeVoltage(-16);
            }
        );
    }

    public Command AutoHubShoot() {
        return runOnce(() -> {
            launcher.setFlywheelVelocity(Constants.LauncherConstants.HUB_RPM);
            launcher.setHoodPosition(Constants.LauncherConstants.HUB_HOOD_ANGLE);
            launcher.runFlyWheel();
            launcher.runHood();
            spindexer.setVoltage(16);
            intake.setIntakeVoltage(-16);

            }
        );
    }

    public Command AutoTowerShoot() {
        return runOnce(() -> {
            launcher.setFlywheelVelocity(Constants.LauncherConstants.TOWER_RPM);
            launcher.setHoodPosition(Constants.LauncherConstants.TOWER_HOOD_ANGLE);
            launcher.runFlyWheel();
            launcher.runHood();
            spindexer.setVoltage(16);
            intake.setIntakeVoltage(-16);

            }
        );
        }

    public Command AutoShuttle() {
        return runOnce(() -> {
            launcher.setFlywheelVelocity(Constants.LauncherConstants.SHUTTLE_AUTO_RPM);
            launcher.setHoodPosition(Constants.LauncherConstants.SHUTTLE_AUTO_START_HOOD_ANGLE);
            launcher.runFlyWheel();
            launcher.runHood();
            spindexer.setVoltage(16);
            intake.setIntakeVoltage(-16);
        } 
        
        );
    }
     
    public Command ExtendIntake() {
        return runOnce(() -> intake.ExtendIntake());
    }

    public Command RetractIntake() {
        return runOnce(() -> intake.RetractIntake());
    }

    public Command StopLauncher() {
        return Commands.runOnce(() -> {
            launcher.stopFlyWheels();
            spindexer.setVoltage(0);
            launcher.setHoodPosition(0);
            launcher.runHood();
        }, launcher, spindexer
        );
    }

    public void StopAll() {
            launcher.stopFlyWheels();
            spindexer.setVoltage(0);
            launcher.setHoodPosition(0);
            launcher.runHood();
            intake.setIntakeVoltage(0);
        }

    public Command AutoAimShoot(){
        // This command enables auto-aim and then runs the LauncherCommand.
        // It is designed to run continuously until cancelled by another command (like AutoAimStop).
        // In PathPlanner, this should be used as a "Parallel" or "Start" event.
        return new LauncherCommand(launcher, spindexer, drivetrain, intake)
            .beforeStarting(new InstantCommand(() -> drivetrain.setAutoAim(true)));
    }

    public Command AutoAimStop(){
        // This command disables auto-aim and stops the launcher command by interrupting it.
        return new InstantCommand(
            () -> drivetrain.setAutoAim(false));
    }

    public Command RunIntake() {
        return Commands.runOnce(() -> intake.setIntakeVoltage(-16));
    }


    }

    
    // public void defenceX() {
    //     drivetrain.getModule(0).apply(
    //         new ModuleRequest().withState( new SwerveModuleState(0, Rotation2d.fromDegrees(105))));
    //     drivetrain.getModule(0).apply(
    //         new ModuleRequest().withState( new SwerveModuleState(0, Rotation2d.fromDegrees(-75))));
    //     drivetrain.getModule(0).apply(
    //         new ModuleRequest().withState( new SwerveModuleState(0, Rotation2d.fromDegrees(15))));
    //     drivetrain.getModule(0).apply(
    //         new ModuleRequest().withState( new SwerveModuleState(0, Rotation2d.fromDegrees(-75))));
        
    //     // drivetrain.configNeutralMode(NeutralModeValue.Brake);
    // }

    
    
    

