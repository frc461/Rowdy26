package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    private final TalonFX FlywheelAKraken = new TalonFX(61);
    private final TalonFX FlywheelBKraken = new TalonFX(62);
    private final TalonFX KickerKraken = new TalonFX(55);
    private final TalonFX HoodKraken = new TalonFX(57);

    private double targetFlywheelRPM = 0.0;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    public Launcher() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        HoodKraken.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;
        HoodKraken.getConfigurator().apply(config);

        FlywheelAKraken.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;
        FlywheelAKraken.getConfigurator().apply(config);

        FlywheelBKraken.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;
        FlywheelBKraken.getConfigurator().apply(config);
    }

    private final VoltageOut voltageControl = new VoltageOut(0);

    public void setVoltage(double volts) {
        FlywheelAKraken.setControl(voltageControl.withOutput(volts));
    }

    public void setFlyWheelBVoltage(double volts) {
        FlywheelBKraken.setControl(voltageControl.withOutput(volts));
    }

    public void setKickerVoltage(double volts) {
        KickerKraken.setControl(voltageControl.withOutput(volts));
        HoodKraken.setControl(voltageControl.withOutput(volts));
    }


    public void setFlywheelVelocity(double RPM) {
        this.targetFlywheelRPM = RPM;
        double rps = RPM / 60.0;

        FlywheelAKraken.setControl(velocityControl.withVelocity(rps) );
        FlywheelBKraken.setControl(velocityControl.withVelocity(rps) );   
    }
    public void setKickerVelocity(double RPM) {
        KickerKraken.setControl(velocityControl.withVelocity(RPM / 60.0));
    }
    public void stopAll() {
        FlywheelAKraken.stopMotor();
        FlywheelBKraken.stopMotor();
        KickerKraken.stopMotor();
        HoodKraken.stopMotor();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Flywheel Actual RPM", FlywheelAKraken.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("Flywheel Target RPM", targetFlywheelRPM);
        SmartDashboard.putNumber("Flywheel Temperature", FlywheelAKraken.getDeviceTemp().getValueAsDouble());

    }
}
