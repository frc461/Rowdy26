package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveCommand extends Swerve implements Subsystem {

    // 1. Instantiate the Field2d object
    private final Field2d m_field = new Field2d();

    public SwerveCommand(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
       
        // 2. Publish the Field2d object to NetworkTables
        SmartDashboard.putData("Field Layout", m_field);
    }

    // ... other constructors ...

    @Override
    public void periodic() {
        // 3. Extract the high-frequency pose and update the field widget
        m_field.setRobotPose(this.getState().Pose);
    }
}



