package frc.robot.subsystems.lights;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

/**
 * Subsystem that controls an addressable LED strip using a CANdle.
 */
public class LEDSubsystem extends SubsystemBase {

    private final CANBus kCANBus = new CANBus("rio");
    private final CANdle m_candle = new CANdle(15, kCANBus);

    private final StrobeAnimation m_slot0Animation = new StrobeAnimation(1234, 0)
        .withSlot(0)
        .withColor(new RGBWColor(10, 249, 12, 0))
        .withFrameRate(Hertz.of(1));

    private final SolidColor[] m_colors = new SolidColor[] {
        new SolidColor(1234, 0), // red example
    };

    public LEDSubsystem() {
        setDefaultCommand(updateLEDs());
    }

    /**
     * Updates the animations and LEDs of the CANdle.
     */
    public Command updateLEDs() {
        return run(() -> {
            for (SolidColor color : m_colors) {
                m_candle.setControl(color);
            }

            // Example animation
            m_candle.setControl(m_slot0Animation);
        });
    }
}