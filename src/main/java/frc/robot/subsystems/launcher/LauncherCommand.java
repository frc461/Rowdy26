package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.Command;

public class LauncherCommand extends Command {
    private final Launcher launcher;
    
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
