package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.localizer.Localizer;
import frc.robot.subsystems.launcher.ShooterSolver;
import frc.robot.subsystems.launcher.ShooterSolver.ShotResult;

public class AimAtHubCommand extends Command {
    private static final double AUTO_AIM_TRANSLATION_SCALE = 0.1;

    private final Swerve drivetrain;
    private final Launcher launcher;
    private final Localizer localizer;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier strafeInput;

    // Swerve request that accepts X/Y velocity but allows us to inject a custom rotational rate
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    // PID Controller specifically for snapping the heading
    private final PIDController turnPID;

    public  AimAtHubCommand(
            Swerve drivetrain, 
            Launcher launcher, 
            Localizer localizer, 
            DoubleSupplier forwardInput, 
            DoubleSupplier strafeInput) {

        this.drivetrain = drivetrain;
        this.launcher = launcher;
        this.localizer = localizer;
        this.forwardInput = forwardInput;
        this.strafeInput = strafeInput;

        // Tune these PID values in SmartDashboard
        // P value defines how aggressively the robot snaps to the angle
        turnPID = new PIDController(0.25, 0.0, 0.005);
        
        // CRITICAL: Tells the PID loop that 360 degrees and 0 degrees are the same point.
        // This prevents the robot from spinning the long way around.
        turnPID.enableContinuousInput(0, 360); 

        // Auto-aim owns drivetrain and launcher while active. Driver translation is
        // still preserved through the supplied joystick inputs below.
        // addRequirements(drivetrain, launcher);
    }

    @Override
    public void execute() {
        // 1. Get the current, most accurate pose from your Localizer
        Pose2d currentPose = drivetrain.getState().Pose;

        // 2. Ask the solver for the math
        ShotResult solution = ShooterSolver.solve(currentPose);

        // 3. Calculate rotational velocity to aim the robot
        // The solver normalizes to 0-360. We must ensure the current pose is also 0-360.
        double currentHeading = MathUtil.inputModulus(currentPose.getRotation().getDegrees(), 0, 360);
        
        double rotationalVelocity = turnPID.calculate(currentHeading, solution.headingDegrees);

        // Clamp the rotational speed to prevent violent spinning
        // (Assuming max angular rate is ~3.0 radians/second)
        rotationalVelocity = MathUtil.clamp(rotationalVelocity, -3.0, 3.0);

        // 4. Command the Swerve Drive
        // Driver controls translation, PID controls rotation toward the hub.
        drivetrain.setControl(
            driveRequest
                .withVelocityX(forwardInput.getAsDouble() * AUTO_AIM_TRANSLATION_SCALE)
                .withVelocityY(strafeInput.getAsDouble() * AUTO_AIM_TRANSLATION_SCALE)
                .withRotationalRate(rotationalVelocity)
        );

        // 5. Command the Launcher
        if (solution.found) {
            launcher.setFlywheelVelocity(-1.0 * solution.rpm);
            launcher.setHoodPosition(Constants.LauncherConstants.AUTO_AIM_HOOD_ANGLE);
            SmartDashboard.putBoolean("Shooter/Solution Found", true);
        } else {
            SmartDashboard.putBoolean("Shooter/Solution Found", false);
        }

        

        // Publish telemetry for tuning
        SmartDashboard.putNumber("Shooter/Target Heading", solution.headingDegrees);
        SmartDashboard.putNumber("Shooter/Current Heading", currentHeading);
        SmartDashboard.putNumber("Shooter/Target RPM", solution.rpm);
    }

    
    // @Override
    // public void end(boolean interrupted) {
    //     // When the driver lets go of the button, stop the launcher
    //     launcher.stopFlyWheels();
    // }
}
