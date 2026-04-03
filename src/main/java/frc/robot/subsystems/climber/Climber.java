package frc.robot.subsystems.climber;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    private final TalonFX ClimbKraken = new TalonFX(0);
    private final TalonFX HookKraken = new TalonFX(0);
    public Climber() {
        ClimbKraken.getConfigurator().apply(new TalonFXConfiguration());
        HookKraken.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration config = new TalonFXConfiguration();
        ClimbKraken.getConfigurator().apply(config); 
        HookKraken.getConfigurator().apply(config);     
    }
    private final VoltageOut voltageControl = new VoltageOut(0);

    public void setVoltage(double volts) {
        ClimbKraken.setControl(voltageControl.withOutput(volts));
        HookKraken.setControl(voltageControl.withOutput(volts));
        
    }

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);


    public void setVelocity(double rotationsPerSecond) {
        ClimbKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
        HookKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
    }
}

