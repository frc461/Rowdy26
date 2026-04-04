    package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;
import frc.robot.subsystems.launcher.Launcher;

import java.util.Optional;

public class ShooterSolver {

    // --- TUNABLE CONSTANTS ---
    public static double EFFICIENCY = 0.63;
    public static double HOOD_ANGLE_DEGREES = 0.0;
    public static final double SHOOTER_HEIGHT_METERS = 0.508; 
    public static final double WHEEL_RADIUS_METERS = 0.0508;
    
    // Robot Geometry (Meters)
    // Based on: Length 32in, Width 26in. Shooter at Back-Right.
    // Local offsets relative to robot center.
    // X is Front/Back, Y is Left/Right.
    // Shooter offset from center (Back edge + inset, Right edge + inset)
    public static final double SHOOTER_LOCAL_X = -0.36; // Approx -14.2 inches
    public static final double SHOOTER_LOCAL_Y = -0.25; // Approx -9.8 inches

    // Field / Target Constants
    public static final double GRAVITY = 9.81;
    public static final double TARGET_HEIGHT_METERS = 1.83; 
    public static final double TARGET_RADIUS_METERS = 0.53; 
    

    // Standard WPILib Field Length
    public static final double FIELD_LENGTH_METERS = 16.5417; 
    
    // Blue Alliance Hub Coordinates
    public static final double BLUE_TARGET_X = 4.65; // 4.62 [m]
    public static final double BLUE_TARGET_Y = 4.35; // 4.02 [m]`   

    // Ball Physics

    public static final double BALL_MASS_KG = 0.215;
    public static final double BALL_RADIUS_METERS = 0.075;
    public static final double DRAG_COEFF = 0.45;

    public static final boolean ENABLE_DRAG = true;

    // Solver Settings
    private static final double DT = 0.01;
    private static final double MAX_TIME = 4.0;

    public final Launcher launcher = new Launcher();

    public static class ShotResult {
        public double headingDegrees;
        public double rpm;
        public boolean found;

        public ShotResult(double heading, double rpm, boolean found) {
            this.headingDegrees = heading;
            this.rpm = rpm;
            this.found = found;
        }
    }

    /**
     * Calculates firing solution based on current robot odometry and alliance.
     * @param robotPose Current Pose2d from the Localizer
     * @return ShotResult containing required Heading and RPM
     */
    public static ShotResult solve(Pose2d robotPose) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        // 1. Determine Target coordinates based on current Alliance
        double currentTargetX = BLUE_TARGET_X;
        double currentTargetY = BLUE_TARGET_Y;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            // Mirror X coordinate for Red alliance (assuming standard WPILib fixed blue origin)
            currentTargetX = FIELD_LENGTH_METERS - BLUE_TARGET_X;
            
            // Note: If the field geometry is perfectly mirrored on the Y axis as well, 
            // you may need to invert the Y coordinate depending on the specific game manual.
            // currentTargetY = FIELD_WIDTH_METERS - BLUE_TARGET_Y; 
        }

        // 2. Iteratively solve Heading (Shooter Offset Compensation)
        double headingRad = 0.0;
        
        for (int i = 0; i < 8; i++) {
            Point shooterPos = calculateShooterPosition(robotX, robotY, headingRad);
            double dx = currentTargetX - shooterPos.x;
            double dy = currentTargetY - shooterPos.y;
            headingRad = Math.atan2(dy, dx);
        }

        double headingDegrees = Math.toDegrees(headingRad);
        headingDegrees = (headingDegrees % 360 + 360) % 360; // Normalize 0-360

        // 3. Binary Search for Required Velocity
        Point finalShooterPos = calculateShooterPosition(robotX, robotY, headingRad);
        double dx = currentTargetX - finalShooterPos.x;
        double dy = currentTargetY - finalShooterPos.y;
        double distanceToTarget = Math.hypot(dx, dy);

        if (distanceToTarget > 2 && distanceToTarget < Units.feetToMeters(14)){//Meters
            Constants.LauncherConstants.AUTO_AIM_HOOD_ANGLE = Constants.LauncherConstants.SIXTY_DEG_HOOD_ANGLE; //far
            HOOD_ANGLE_DEGREES = 60.0;
        }else if(distanceToTarget > Units.feetToMeters(14)){
            Constants.LauncherConstants.AUTO_AIM_HOOD_ANGLE = Constants.LauncherConstants.FIFTY_FIVE_DEG_HOOD_ANGLE; //really far
            HOOD_ANGLE_DEGREES = 60.0;
        }else{
            Constants.LauncherConstants.AUTO_AIM_HOOD_ANGLE = Constants.LauncherConstants.SEVENTY_DEG_HOOD_ANGLE;//close
            HOOD_ANGLE_DEGREES = 70.0;
        }
        
        double hoodRad = Math.toRadians(HOOD_ANGLE_DEGREES);
        
        double minV = 5.0;
        double maxV = 20.0;
        double bestV = 0.0;
        double minError = 999.0;

        for (int i = 0; i < 12; i++) {
            double v = (minV + maxV) / 2.0;
            
            double vZ = v * Math.sin(hoodRad);
            double vHorizontal = v * Math.cos(hoodRad);
            
            SimResult sim = simulateShot(distanceToTarget, SHOOTER_HEIGHT_METERS, vHorizontal, vZ);
            double error = sim.heightAtTarget - TARGET_HEIGHT_METERS;

            if (Math.abs(error) < Math.abs(minError)) {
                minError = error;
                bestV = v;
            }

            if (error < 0) {
                minV = v; // Shot too low
            } else {
                maxV = v; // Shot too high
            }
        }

        // 4. Convert Velocity to RPM
        double rpm = calculateRPM(bestV);
        boolean found = Math.abs(minError) < 0.5;

        return new ShotResult(headingDegrees, rpm, found);
    }

    private static SimResult simulateShot(double targetDist, double startZ, double vHoriz, double vZ) {
        double x = 0; 
        double z = startZ; 
        double vx = vHoriz;
        double vz_vel = vZ;

        double area = Math.PI * BALL_RADIUS_METERS * BALL_RADIUS_METERS;
        double k = (0.5 * 1.225 * area * DRAG_COEFF) / BALL_MASS_KG;

        for (double t = 0; t < MAX_TIME; t += DT) {
            double vMag = Math.sqrt(vx * vx + vz_vel * vz_vel);

            double ax = 0;
            double az = -GRAVITY;

            if (ENABLE_DRAG) {
                ax = -k * vMag * vx;
                az -= (k * vMag * vz_vel);
            }

            x += vx * DT;
            z += vz_vel * DT;
            vx += ax * DT;
            vz_vel += az * DT;

            if (x >= targetDist) {
                return new SimResult(z);
            }

            if (z < 0) break; 
        }

        return new SimResult(-100.0); 
    }

    private static Point calculateShooterPosition(double robotX, double robotY, double headingRad) {
        double offsetX = SHOOTER_LOCAL_X * Math.cos(headingRad) - SHOOTER_LOCAL_Y * Math.sin(headingRad);
        double offsetY = SHOOTER_LOCAL_X * Math.sin(headingRad) + SHOOTER_LOCAL_Y * Math.cos(headingRad);

        return new Point(robotX + offsetX, robotY + offsetY);
    }

    private static double calculateRPM(double velocity) {
        double vTangential = velocity / EFFICIENCY;
        double circumference = 2 * Math.PI * WHEEL_RADIUS_METERS;
        return (vTangential / circumference) * 60.0;
    }

    private static class Point {
        double x, y;
        Point(double x, double y) { this.x = x; this.y = y; }
    }

    private static class SimResult {
        double heightAtTarget;
        SimResult(double h) { this.heightAtTarget = h; }
    }
}
