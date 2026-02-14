package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.InvertedValue;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    private final TalonFX FlywheelAKraken = new TalonFX(61);
    private final TalonFX FlywheelBKraken = new TalonFX(62);
    private final TalonFX KickerKraken = new TalonFX(55);
    private final TalonFX HoodKraken = new TalonFX(57);

    private double rotationsPerMinute;

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


        rotationsPerMinute = 0;
        SmartDashboard.setDefaultNumber("RPM", 0);
    }

    private final VoltageOut voltageControl = new VoltageOut(0);

    public void setFlyWheelAVoltage(double volts) {
        FlywheelAKraken.setControl(voltageControl.withOutput(volts));
    }

    public void setFlyWheelBVoltage(double volts) {
        FlywheelBKraken.setControl(voltageControl.withOutput(volts));
    }

    public void setKickerVoltage(double volts) {
        KickerKraken.setControl(voltageControl.withOutput(volts));
    }

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);


    public void setVelocity(double rotationsPerSecond) {
        FlywheelAKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
        FlywheelBKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
        HoodKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
        KickerKraken.setControl(
                velocityControl.withVelocity(rotationsPerSecond)
        );
    }

    public void setSpeedFunction() {
        FlywheelAKraken.setControl(
            velocityControl.withVelocity(rotationsPerMinute / 60.0)
        );
        FlywheelBKraken.setControl(
            velocityControl.withVelocity(rotationsPerMinute / 60.0)
        );
        KickerKraken.setControl(
                velocityControl.withVelocity(rotationsPerMinute / 60.0)
        );
    }

    public void periodic() {
        rotationsPerMinute = SmartDashboard.getNumber("RPM", 0);
    }
}
