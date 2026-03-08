package frc.robot.subsystems.localizer;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.vision.Vision;

public class Localizer extends SubsystemBase {

    private final Swerve swerve;
    private final Vision vision;
    public Field2d fieldOdom;

    public Localizer(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
        fieldOdom = new Field2d();

        SmartDashboard.putData("Field2d Pose", fieldOdom);
    }

    @Override
    public void periodic() {
        // 1. Tell the Vision subsystem to read all cameras and feed the data to CTRE
        vision.updateVisionPoses(swerve.getState().Pose);

        // 2. Update the dashboard field with the fused CTRE pose
        fieldOdom.setRobotPose(swerve.getState().Pose);
        
        // 3. Draw the GHOST robots (the raw camera estimates)
        // This automatically creates a new layer on Elastic called "Camera Ghosts"
        fieldOdom.getObject("Vision Poses").setPoses(vision.getVisionRobotPoses());
    }
}
