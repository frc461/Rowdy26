package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LauncherCommand extends Command {
    private final Launcher launcher;

    private static final String KEY_FLY_RPM = "Launcher RPM";
    private static final String KEY_HOOD_ANGLE = "Hood ANG" ;
    
    public LauncherCommand(Launcher launcher) {
        this.launcher = launcher;
        addRequirements(launcher);

        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stopFlyWheels();
    }
}
