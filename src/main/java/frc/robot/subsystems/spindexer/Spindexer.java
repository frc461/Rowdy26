package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Spindexer {
    private final TalonFX Kraken = new TalonFX(0);

    public Spindexer() {
        Kraken.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        Kraken.getConfigurator().apply(config);
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

 