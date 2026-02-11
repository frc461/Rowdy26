package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.spindexer.Spindexer;

public class Intake {
    private final TalonFX Kraken = new TalonFX(0);

    public Intake() {
        Kraken.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration config = new TalonFXConfiguration();
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
