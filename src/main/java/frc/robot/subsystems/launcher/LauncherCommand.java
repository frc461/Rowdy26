package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LauncherCommand extends Command {
    private final Launcher launcher;

    private static final String KEY_FLY_RPM = "Launcher RPM";
    private static final String KEY_KICKER_RPM = "Kicker RPM";

    public LauncherCommand(Launcher launcher) {
        this.launcher = launcher;
        addRequirements(launcher);

        SmartDashboard.putNumber(KEY_FLY_RPM, 0.0);
        SmartDashboard.putNumber(KEY_KICKER_RPM, 0.0);
    }

    @Override
    public void execute() {
        double launcherRPM = SmartDashboard.getNumber(KEY_FLY_RPM, 0.0);
        double kickerRPM = SmartDashboard.getNumber(KEY_KICKER_RPM, 0.0);

        launcherRPM = (launcherRPM > 6000) ? 6000 : ((launcherRPM < 0) ? 0 : launcherRPM);
        kickerRPM = (kickerRPM > 6000) ? 6000 : ((kickerRPM < 0) ? 0 : kickerRPM);

        launcher.setFlywheelAVelocity(launcherRPM);
        launcher.setKickerVelocity(kickerRPM);
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stopAll();
    }
}
