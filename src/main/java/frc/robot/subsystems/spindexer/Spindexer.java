package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Spindexer extends SubsystemBase{
    private final TalonFX SpindexerKraken = new TalonFX(50);

    public Spindexer() {
        SpindexerKraken.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        SpindexerKraken.getConfigurator().apply(config);

        SmartDashboard.putNumber("Spindexer Supply", SpindexerKraken.getSupplyCurrent().getValueAsDouble());
    }

    private final VoltageOut voltageControl = new VoltageOut(0);

    public void setVoltage(double volts) {
        SpindexerKraken.setControl(voltageControl.withOutput(volts));
    }

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);


    public void setVelocity(double rotationsPerSecond) {
        SpindexerKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
    }
}

 