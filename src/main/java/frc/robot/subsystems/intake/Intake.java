package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final TalonFX DeployKraken = new TalonFX(52); 
    private final TalonFX IntakeKraken = new TalonFX(56); 
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);


    public Intake() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        DeployKraken.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = 1;
        config.Slot0.kI = 0.2;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.15;
        DeployKraken.getConfigurator().apply(config);
        DeployKraken.setPosition(0);


        TalonFXConfiguration intakeconfig = new TalonFXConfiguration();
        IntakeKraken.getConfigurator().apply(new TalonFXConfiguration());
        intakeconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeconfig.CurrentLimits.SupplyCurrentLimit = 60;
        intakeconfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeconfig.CurrentLimits.StatorCurrentLimit = 80;

        IntakeKraken.getConfigurator().apply(intakeconfig);
    }

    private final DigitalInput RightForwardLimitSwitch = new DigitalInput(0);
    private final DigitalInput LeftForwardLimitSwitch = new DigitalInput(1);
    private final DigitalInput RearLimitSwitch = new DigitalInput(2);

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

    public void CheckRearLimitSwitch() {
        if (!RearLimitSwitch.get()) {
            DeployKraken.setPosition(0);
        }
    }

    public void ResetExtension() {
        if (!RightForwardLimitSwitch.get() && !LeftForwardLimitSwitch.get()) {
            DeployKraken.setPosition(-70);
        }
    }

    public void ExtendIntake() {
        DeployKraken.setControl(positionControl.withPosition(-70)
        .withLimitForwardMotion(!RightForwardLimitSwitch.get() && !LeftForwardLimitSwitch.get())
        );
    }

    public void RetractIntake() {
        DeployKraken.setControl(positionControl.withPosition(0));
    }
     @Override
    public void periodic() {

        SmartDashboard.putBoolean("Right Forward Limit Switch", !RightForwardLimitSwitch.get());
        SmartDashboard.putBoolean("Left Forward Limit Switch", !LeftForwardLimitSwitch.get());
        SmartDashboard.putBoolean("Rear Limit Switch", !RearLimitSwitch.get());
        SmartDashboard.putNumber("Intake Position", DeployKraken.getPosition().getValueAsDouble());
        CheckRearLimitSwitch();
        ResetExtension();
    }
}
