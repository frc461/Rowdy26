package frc.robot.subsystems.lights;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/**
 * Subsystem that controls an addressable LED strip using a CANdle.
 */
public class LEDSubsystem extends SubsystemBase {

    private final CANBus kCANBus = new CANBus("rio");
    private final CANdle m_candle = new CANdle(15, kCANBus);

    private final StrobeAnimation m_flashingGreen = new StrobeAnimation(0, 0)
        .withColor(new RGBWColor(0, 255, 0, 0))
        .withFrameRate(Hertz.of(2)); // Flashing approx 2 times a second
    private final SolidColor m_solidGreen = new SolidColor(0, 0)
        .withColor(new RGBWColor(0, 255, 0, 0));
    private final SolidColor m_solidRed = new SolidColor(0, 0)
        .withColor(new RGBWColor(255, 0, 0, 0));
    
    // For simulated test mode
    private double m_simulatedStartTime = 0;

    public LEDSubsystem() {
        setDefaultCommand(updateLEDsFMS());
    }

    /**
     * Determines whether the hub is active based on match time and alliance.
     */
    private boolean isOurHubActive(double matchTime) {
        String gameData = DriverStation.getGameSpecificMessage();
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (gameData == null || gameData.isEmpty() || alliance.isEmpty() || DriverStation.isAutonomousEnabled()) {
            return true;
        }

        boolean weAreInactiveFirst = false;
        char firstInactive = gameData.charAt(0);
        
        if (firstInactive == 'R' && alliance.get() == Alliance.Red) {
            weAreInactiveFirst = true;
        } else if (firstInactive == 'B' && alliance.get() == Alliance.Blue) {
            weAreInactiveFirst = true;
        }
        
        boolean shift1Active = !weAreInactiveFirst;
        
        if (matchTime > 130.0) return true;
        if (matchTime > 105.0) return shift1Active;
        if (matchTime > 80.0) return !shift1Active;
        if (matchTime > 55.0) return shift1Active;
        if (matchTime > 30.0) return !shift1Active;
        return true;
    }

    /**
     * Calculates time remaining in the current shift.
     */
    private double getShiftTimeRemaining(double matchTime) {
        if (matchTime > 130) return matchTime - 130;
        if (matchTime > 105) return matchTime - 105;
        if (matchTime > 80) return matchTime - 80;
        if (matchTime > 55) return matchTime - 55;
        if (matchTime > 30) return matchTime - 30;
        return matchTime;
    }

    /**
     * Core logic to set LEDs based on current match time.
     */
    private void applyLEDState(double matchTime) {
        boolean active = isOurHubActive(matchTime);
        double shiftRemaining = getShiftTimeRemaining(matchTime);

        if (active) {
            m_candle.setControl(m_solidGreen);
        } else if (shiftRemaining <= 5.0) {
            m_candle.setControl(m_flashingGreen);
        } else {
            m_candle.setControl(m_solidRed);
        }
    }

    /**
     * Command that updates LEDs using real FMS DriverStation Match Time.
     */
    public Command updateLEDsFMS() {
        return run(() -> applyLEDState(DriverStation.getMatchTime()));
    }

    /**
     * Command that simulates a 135-second teleop practice match timing internally, 
     * ignoring the real FMS timer.
     */
    // public Command updateLEDsSimulated() {
    //     return runOnce(() -> m_simulatedStartTime = Timer.getFPGATimestamp())
    //         .andThen(run(() -> {
    //             double elapsed = Timer.getFPGATimestamp() - m_simulatedStartTime;
    //             double simulatedMatchTime = Math.max(0, 135.0 - elapsed);
    //             applyLEDState(simulatedMatchTime);
    //         }));
    // }
}