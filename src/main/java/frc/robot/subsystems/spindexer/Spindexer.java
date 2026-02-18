package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Spindexer extends SubsystemBase{
    private final TalonFX SpindexerKraken = new TalonFX(50);
    private final TalonFX KickerKraken = new TalonFX(55);//Kicker

    public Spindexer() {
        SpindexerKraken.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        SpindexerKraken.getConfigurator().apply(config);
    }

    private final VoltageOut voltageControl = new VoltageOut(0);

    public void setVoltage(double volts) {
        SpindexerKraken.setControl(voltageControl.withOutput(volts));
        final VelocityVoltage velocityControl = new VelocityVoltage(0);
    }

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);


    public void setVelocity(double rotationsPerSecond) {
        SpindexerKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
    }
    public void KickerFollowSpindexer() {
      KickerKraken.setControl(new Follower(SpindexerKraken.getDeviceID(), MotorAlignmentValue.Aligned));
    }

}

 