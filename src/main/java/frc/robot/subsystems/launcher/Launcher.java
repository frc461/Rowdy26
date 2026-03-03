package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Launcher extends SubsystemBase {
    private final TalonFX FlywheelAKraken = new TalonFX(61);
    private final TalonFX FlywheelBKraken = new TalonFX(62);
    private final TalonFX KickerKraken = new TalonFX(55);
    private final TalonFX HoodKraken = new TalonFX(57);

    private double targetFlywheelRPM = 0.0;
    private double targetHoodPosition = 0.0;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private final PositionVoltage positionControl = new PositionVoltage(0);

    private final CANcoder hoodAbsoluteEncoder = new CANcoder(58);
    


    public Launcher() {

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = Constants.LauncherConstants.ABSOLUTE_ENCODER_OFFSET;
        hoodAbsoluteEncoder.getConfigurator().apply(encoderConfig);

        TalonFXConfiguration config = new TalonFXConfiguration();
        HoodKraken.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = 5;
        config.Slot0.kI = 0.2;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.8;
        // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // config.Feedback.FeedbackRemoteSensorID = hoodAbsoluteEncoder.getDeviceID();
        // config.Feedback.SensorToMechanismRatio = 1.0;
        // config.Feedback.RotorToSensorRatio = 4.0;
        HoodKraken.getConfigurator().apply(config);
        HoodKraken.setPosition(0);
        


        FlywheelAKraken.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = .4;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.2056;
        FlywheelAKraken.getConfigurator().apply(config);

        FlywheelBKraken.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = .4;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.2056;
        FlywheelBKraken.getConfigurator().apply(config);

        KickerKraken.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        KickerKraken.setControl(
            new Follower(50, MotorAlignmentValue.Aligned)
        );

        SmartDashboard.putNumber(KEY_HOOD_ANGLE, 0.0);
        SmartDashboard.putNumber(KEY_FLY_RPM, 0.0);

        // SmartDashboard.putNumber("Tuning Kp", lastP);
        // SmartDashboard.putNumber("Tuning Ki", lastI);
        // SmartDashboard.putNumber("Tuning Kd", lastD);        
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

    }

    public void setFlywheelVelocity(double RPM) {
        this.targetFlywheelRPM = RPM;
        
    }
    public void runFlyWheel() {
        double rps = targetFlywheelRPM / 60.0;
        FlywheelAKraken.setControl(velocityControl.withVelocity(rps) );
        FlywheelBKraken.setControl(velocityControl.withVelocity(rps) );   
    }

    public void setHoodPosition(double pose) {
        this.targetHoodPosition = convertHoodPosition(pose);
        
    }

    public double convertHoodPosition(double pose) {
        double currentHoodEncoderPose = hoodAbsoluteEncoder.getPosition().getValueAsDouble();
        double encoderPose = pose / Constants.LauncherConstants.ENCODER_CONVERSION;
        double currentHoodMotorPose = HoodKraken.getPosition().getValueAsDouble();
        return ((currentHoodEncoderPose + encoderPose) * Constants.LauncherConstants.ENCODER_CONVERSION) + currentHoodMotorPose;
    }



    public void runHood() {
        HoodKraken.setControl(positionControl.withPosition(targetHoodPosition));
    }
    
    
    public void setKickerVelocity(double RPM) {
        KickerKraken.setControl(velocityControl.withVelocity(RPM / 60.0));
    }
    public void stopFlyWheels() {
        FlywheelAKraken.stopMotor();
        FlywheelBKraken.stopMotor();
    }

    private static final String KEY_FLY_RPM = "Launcher RPM";
    private static final String KEY_HOOD_ANGLE = "Hood ANG" ;
    

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Flywheel Actual RPM", FlywheelAKraken.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("Flywheel Target RPM", targetFlywheelRPM);
        SmartDashboard.putNumber("Flywheel Temperature", FlywheelAKraken.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Hood Position", HoodKraken.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Encoder Position", hoodAbsoluteEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("convertHoodPosition", convertHoodPosition(1.25));

        
        double hoodAngle = SmartDashboard.getNumber(KEY_HOOD_ANGLE, 0.0);
        double launcherRPM = SmartDashboard.getNumber(KEY_FLY_RPM, 0.0);
       
        
        // setFlywheelVelocity(launcherRPM);
        // setHoodPosition(hoodAngle);
    }
}
