package frc.robot.subsystems.lights;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// this is the set that will flash via match time when played on a practice match

@SuppressWarnings("removal")
public class Lights extends SubsystemBase {

    private final com.ctre.phoenix.led.CANdle candle = new com.ctre.phoenix.led.CANdle(0, "rio");

    private boolean flashState = false;
    private double lastFlashTime = 0;

    public Lights() {
        com.ctre.phoenix.led.CANdleConfiguration config = new com.ctre.phoenix.led.CANdleConfiguration();
        config.stripType = com.ctre.phoenix.led.CANdle.LEDStripType.RGB;
        config.brightnessScalar = 0.5;
        candle.configAllSettings(config);
    }

    @Override
    public void periodic() {

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        
        boolean isHubActive;

        // SAFETY FALLBACK (practice mode or no FMS)
        if (matchTime < 0) {
            candle.setLEDs(0, 0, 255); // Blue idle
            return;
        }

        // Default active if missing data
        if (gameData == null || gameData.isEmpty() || alliance.isEmpty() || DriverStation.isAutonomousEnabled()) {
            isHubActive = true;
        } else {
            boolean weAreInactiveFirst = false;
            char firstInactive = gameData.charAt(0);

            if (firstInactive == 'R' && alliance.get() == Alliance.Red) {
                weAreInactiveFirst = true;
            } else if (firstInactive == 'B' && alliance.get() == Alliance.Blue) {
                weAreInactiveFirst = true;
            }

            boolean shift1Active = !weAreInactiveFirst;

            // SAME SHIFT LOGIC
            if (matchTime > 130.0) {
                isHubActive = true;
            } else if (matchTime > 105.0) {
                isHubActive = shift1Active;
            } else if (matchTime > 80.0) {
                isHubActive = !shift1Active;
            } else if (matchTime > 55.0) {
                isHubActive = shift1Active;
            } else if (matchTime > 30.0) {
                isHubActive = !shift1Active;
            } else {
                isHubActive = true;
            }
        }

        double shiftTimeRemaining = calculateShiftTime(matchTime);

        updateLEDs(shiftTimeRemaining, isHubActive);
    }

    private void updateLEDs(double shiftTimeRemaining, boolean isActive) {
        double currentTime = Timer.getFPGATimestamp();

        int r, g, b;

        if (isActive) {
            r = 0; g = 255; b = 0;

        } else if (shiftTimeRemaining <= 2.0) {
            r = 255; g = 200; b = 0;

        } else if (shiftTimeRemaining <= 5.0) {
            if (currentTime - lastFlashTime > 0.2) {
                flashState = !flashState;
                lastFlashTime = currentTime;
            }

            if (flashState) {
                r = 255; g = 200; b = 0;
            } else {
                r = 0; g = 0; b = 0;
            }

        } else {
            r = 0; g = 0; b = 0;
        }

        candle.setLEDs(r, g, b);
    }

    private double calculateShiftTime(double time) {
        if (time > 130) return time - 130;
        if (time > 105) return time - 105;
        if (time > 80) return time - 80;
        if (time > 55) return time - 55;
        if (time > 30) return time - 30;
        return time;
    }
}
