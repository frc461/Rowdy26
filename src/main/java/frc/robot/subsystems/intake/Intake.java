package frc.robot.subsystems.intake;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.spindexer.Spindexer;

public class Intake extends SubsystemBase{
    private final TalonFX DeployKraken = new TalonFX(52);
    private final TalonFX IntakeKraken = new TalonFX(56);
public Intake() {
        DeployKraken.getConfigurator().apply(new TalonFXConfiguration());
        IntakeKraken.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration config = new TalonFXConfiguration();
        DeployKraken.getConfigurator().apply(config); 
        IntakeKraken.getConfigurator().apply(config);     
    }
    private final VoltageOut voltageControl = new VoltageOut(0);

    public void setIntakeVoltage(double volts) {
        IntakeKraken.setControl(voltageControl.withOutput(volts));
    }

    public void setDeployVoltage(double volts) {
        DeployKraken.setControl(voltageControl.withOutput(volts));
    }

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);


    public void setVelocity(double rotationsPerSecond) {
        DeployKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
        IntakeKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
    }
}
