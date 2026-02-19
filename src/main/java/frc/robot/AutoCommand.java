package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.spindexer.Spindexer;

public class AutoCommand extends SubsystemBase {

    private final Launcher launcher = new Launcher();
    private final Spindexer spindexer = new Spindexer();

    public Command AutoTowerShoot() {
        return runOnce(() -> {
            launcher.setFlywheelVelocity(-2250);
            launcher.setHoodPosition(1.25);
            spindexer.setVoltage(16);
            launcher.runFlyWheel();
            launcher.runHood();
            }
        );
    }

    public Command AutoHubShoot() {
        return runOnce(() -> {
            launcher.setFlywheelVelocity(-1950);
            launcher.setHoodPosition(0);
            spindexer.setVoltage(16);
            launcher.runFlyWheel();
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
        }
        );
    }
    
    
}
