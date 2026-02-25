package frc.robot.subsystems.hubState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class HubState extends SubsystemBase {

    //public HubTelemetry() {
        //hub active and shift time remaining
    //}

    @Override
    public void periodic() {
        // 1. Get the Game Specific Data (FMS data for which alliance's hub is inactive first)
        String gameData = DriverStation.getGameSpecificMessage();
        
        // 2. Get our current alliance
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        
        // Write the raw FMS data to Elastic
        SmartDashboard.putString("FMS/Inactive First Alliance", gameData.isEmpty() ? "Waiting..." : gameData);
        if (alliance.isPresent()) {
            SmartDashboard.putString("FMS/Our Alliance", alliance.get().name());
        }

        // 3. Determine if our shift is up (Hub is active) and write to Elastic
        boolean active = isOurHubActive(gameData, alliance);
        SmartDashboard.putBoolean("FMS/Is Our Hub Active", active);
    }

    /**
     * Calculates if our alliance's hub is currently active based on 2026 REBUILT game rules.
     */
    private boolean isOurHubActive(String gameData, Optional<DriverStation.Alliance> alliance) {
        // If we have no alliance or are not enabled, the hub isn't active/relevant
        if (alliance.isEmpty() || !DriverStation.isEnabled()) {
            return false;
        }

        // Hub is always active for both alliances during Autonomous
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }

        // If we are in teleop but don't have game data yet, assume active
        if (gameData.isEmpty()) {
            return true;
        }

        // Determine if OUR alliance is the one that goes inactive first
        boolean ourHubInactiveFirst = false;
        char inactiveFirstChar = gameData.charAt(0);
        
        if (alliance.get() == DriverStation.Alliance.Red && inactiveFirstChar == 'R') {
            ourHubInactiveFirst = true;
        } else if (alliance.get() == DriverStation.Alliance.Blue && inactiveFirstChar == 'B') {
            ourHubInactiveFirst = true;
        }

        // Get the approximate match time remaining from the FMS
        double matchTime = DriverStation.getMatchTime();

        // 2026 REBUILT Hub Shift Timing Logic
        if (matchTime > 130) {
            // Transition shift: Hub is active for both alliances
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return !ourHubInactiveFirst;
        } else if (matchTime > 80) {
            // Shift 2
            return ourHubInactiveFirst;
        } else if (matchTime > 55) {
            // Shift 3
            return !ourHubInactiveFirst;
        } else if (matchTime > 30) {
            // Shift 4
            return ourHubInactiveFirst;
        } else {
            // End Game (30 seconds and under): Hub is always active
            return true;
        }
    }
}