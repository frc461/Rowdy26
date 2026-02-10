package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Launcher {
    private final TalonFX Kraken = new TalonFX(0);

    public Launcher() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;
    }

    private final VoltageOut voltageControl = new VoltageOut(0);

    public void setVoltage(double volts) {
        Kraken.setControl(voltageControl.withOutput(volts));
        final VelocityVoltage velocityControl = new VelocityVoltage(0);
    }

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);


    public void setVelocity(double rotationsPerSecond) {
        Kraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
    }
}
