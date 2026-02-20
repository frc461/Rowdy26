package frc.robot.subsystems.intake;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.spindexer.Spindexer;

public class Intake extends SubsystemBase{ //Always use extends SubsystemBase to make sure you can refrence this class in Robot Container 
    private final TalonFX DeployKraken = new TalonFX(52); //giving the intake motors the correct CAN ID
    private final TalonFX IntakeKraken = new TalonFX(56); // CAN ID's are taken form Phoneix Tuner
    private final VoltageOut voltageControl = new VoltageOut(0);

    public void setIntakeVoltage(double volts) {
        IntakeKraken.setControl(voltageControl.withOutput(volts));
    }

    public void setDeployVoltage(double volts) {
        DeployKraken.setControl(voltageControl.withOutput(volts));
    }
    //setting it so that all you need to do is give the voltage number in Robot Container and it will run the set Motor
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);


    public void setVelocity(double rotationsPerSecond) {
        DeployKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
        IntakeKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
    }// setiing the correct amount of rotations needed for each motor, change the values if needed in Robot Container just like Voltage Control
}
