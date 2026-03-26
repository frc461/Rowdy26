package frc.robot.subsystems.hubState;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubState extends SubsystemBase {
    
    private boolean isHubActive = true; 
    private String dashColor = "RED"; 
    private boolean winAuto; 
    
    public void MatchStateSubsystem() {
        // Initialize Dashboard variables
        SmartDashboard.putBoolean("Is Hub Active", true);
        SmartDashboard.putNumber("Shift Time Remaining", 0.0);
    }

    @Override
    public void periodic() {
        // 1. Get Match Time (Approximate time remaining in Teleop)
        double matchTime = DriverStation.getMatchTime();
        
        // 2. Get Game Data ('R' or 'B') from FMS
        String gameData = DriverStation.getGameSpecificMessage();
        
        // 3. Get Our Alliance Color
        Optional<Alliance> alliance = DriverStation.getAlliance();
        
        // SAFETY: If we are missing data or are in Auto, the Hub is always active.
        if (gameData == null || gameData.isEmpty() || alliance.isEmpty() || DriverStation.isAutonomousEnabled()) {
            isHubActive = true;
        } else {
            // Figure out if our alliance's Hub is the one turned off in Shift 1
            boolean weAreInactiveFirst = false;
            char firstInactive = gameData.charAt(0);
            
            if (firstInactive == 'R' && alliance.get() == Alliance.Red) {
                weAreInactiveFirst = true;
            } else if (firstInactive == 'B' && alliance.get() == Alliance.Blue) {
                weAreInactiveFirst = true;
            }
            winAuto = weAreInactiveFirst;
            
            boolean shift1Active = !weAreInactiveFirst;
            
            // 4. Apply the 2026 Shift Schedule (Table 6-2)
            if (matchTime > 130.0) {
                isHubActive = true;            // Transition Shift (Both Active)
            } else if (matchTime > 105.0) {
                isHubActive = shift1Active;    // Shift 1
            } else if (matchTime > 80.0) {
                isHubActive = !shift1Active;   // Shift 2 (Alternates)
            } else if (matchTime > 55.0) {
                isHubActive = shift1Active;    // Shift 3 (Alternates)
            } else if (matchTime > 30.0) {
                isHubActive = !shift1Active;   // Shift 4 (Alternates)
            } else {
                isHubActive = true;            // End Game (Both Active)
            }
        }
        
        double shiftTimeRemaining = calculateShiftTime(matchTime);
        //boolean willBeActiveNext = !isHubActive; // If not active now, it will be next

        if (isHubActive || shiftTimeRemaining < 2.0) {
        // CURRENTLY ACTIVE: Solid Green
        dashColor = "#28A745";
        } else if ( shiftTimeRemaining < 7.0) {
        // NOT ACTIVE YET, BUT SHIFT STARTS IN < 2 SECONDS: Yellow/Warning
        dashColor = "#FFC107";
        } else {
        // INACTIVE: Solid Red
        dashColor = "#DC3545";
        }

        // 5. Publish to SmartDashboard for Elastic
        SmartDashboard.putString("Action Color", dashColor);
        SmartDashboard.putBoolean("Shoot Now", isHubActive);

        SmartDashboard.putBoolean("Is Hub Active", isHubActive);
        SmartDashboard.putNumber("Shift Time Remaining", calculateShiftTime(matchTime));
        SmartDashboard.putString("Current Shift", currentShift(matchTime));
        SmartDashboard.putBoolean("Win Auto", winAuto);

    }
    
    /**
     * Calculates how many seconds are left before the Hub state flips.
     */
    private double calculateShiftTime(double time) {
        if (time > 130) return time - 130; // 10s Transition
        if (time > 105) return time - 105; // 25s Shift 1
        if (time > 80) return time - 80;   // 25s Shift 2
        if (time > 55) return time - 55;   // 25s Shift 3
        if (time > 30) return time - 30;   // 25s Shift 4
        return time;                       // 30s End Game
    }

    private String currentShift(double time){
        if (time > 130) return "Transition";
        if (time > 105) return "Shift 1";
        if (time > 80) return "Shift 2";
        if (time > 55) return "Shift 3";
        if (time > 30) return "Shift 4";
        return "End Game";
    }

    /**
     * Call this in your shooting command to prevent firing into a dead Hub!
     */
    public boolean canScore() {
        return isHubActive;
    }
    // Add this to your class variables

    // ... (Keep your existing matchTime and shift logic here) ...
}
