package frc.robot;

import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.intake.Intake;

public class AutoCommand extends SubsystemBase {

    private Launcher launcher;
    private Spindexer spindexer;
    private Intake intake;

    public AutoCommand(Launcher m_Launcher, Spindexer m_Spindexer, Intake m_Intake)
    {
        launcher = m_Launcher;
        spindexer = m_Spindexer;
        intake = m_Intake;
    }

    public Command AutoHumanPlayerShoot() {
        return runOnce(() -> {
            launcher.setFlywheelVelocity(Constants.LauncherConstants.TEMP_AUTO_RPM);
            launcher.setHoodPosition(Constants.LauncherConstants.TEMP_AUTO_START_HOOD_ANGLE);
            spindexer.setVoltage(16);
            launcher.runFlyWheel();
            launcher.runHood();
            intake.setIntakeVoltage(-16);
            }
        );
    }

    public Command AutoTrenchShoot() {
        return runOnce(() -> {
            launcher.setFlywheelVelocity(Constants.LauncherConstants.TRENCH_AUTO_RPM);
            launcher.setHoodPosition(Constants.LauncherConstants.TRENCH_AUTO_START_HOOD_ANGLE);
            spindexer.setVoltage(16);
            launcher.runFlyWheel();
            launcher.runHood();
            intake.setIntakeVoltage(-16);
            }
        );
    }

    public Command AutoHubShoot() {
        return runOnce(() -> {
            launcher.setFlywheelVelocity(Constants.LauncherConstants.HUB_RPM);
            launcher.setHoodPosition(Constants.LauncherConstants.HUB_HOOD_ANGLE);
            spindexer.setVoltage(16);
            launcher.runFlyWheel();
            launcher.runHood();
            intake.setIntakeVoltage(-16);

            }
        );
    }

    public Command AutoTowerShoot() {
        return runOnce(() -> {
            launcher.setFlywheelVelocity(Constants.LauncherConstants.TOWER_RPM);
            launcher.setHoodPosition(Constants.LauncherConstants.TOWER_HOOD_ANGLE);
            spindexer.setVoltage(16);
            launcher.runFlyWheel();
            launcher.runHood();
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
        return runOnce(() -> {
            launcher.stopFlyWheels();
            spindexer.setVoltage(0);
            launcher.setHoodPosition(0);
            launcher.runHood();
        }
        );
    }

    public Command StopAll() {
        return runOnce(() -> {
            launcher.stopFlyWheels();
            spindexer.setVoltage(0);
            launcher.setHoodPosition(0);
            launcher.runHood();
            intake.setIntakeVoltage(0);
        }
        );
    }
    
    
}
