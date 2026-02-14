package frc.robot.subsystems.launcher;

import java.util.ArrayList;
import java.util.List;
import frc.robot.constants.Constants;

public class LauncherMath {

    private static final double GRAVITY = 9.80665;
    private static final double AIR_DENSITY = 1.225; 
    private static final double IN_TO_M = 0.0254;

    // --- Data Structures (Value Objects) ---

    public record Vector3(double x, double y, double z) {}
    public record Vector2(double x, double y) {}
    public record Velocity3(double vx, double vy, double vz) {}

    public record BallParams(
        double radius,
        double dragCoefficient,
        double mass,
        double shooterWheelRadius,
        double flywheelEfficiency
    ) {}

    public record RobotState(
        double x, double y, double heading,
        double vx, double vy,
        double length, double width,
        double shooterHeight,
        double hoodAngle
    ) {}

    public record SimulationResult(
        boolean hit,
        double verticalError,
        double heightAtTarget
    ) {}

    public record FiringSolution(
        double shotRPM,
        double launchVelocity,
        double turretHeading,
        boolean isSolutionFound
    ) {}

    // --- Core Physics & Logic ---

    /**
     * Projects the ball's path using Euler Integration. 
     * Treats the target as a vertical cylinder to detect the "crossing" point.
     */
    public static SimulationResult simulateTrajectory(
        Vector3 start,
        Velocity3 velocity,
        Vector2 targetPos,
        BallParams params
    ) {
        double dt = 0.01;
        double maxTime = 4.0;

        double x = start.x();
        double y = start.y();
        double z = start.z();
        double vx = velocity.vx();
        double vy = velocity.vy();
        double vz = velocity.vz();

        double targetDistXY = Math.hypot(targetPos.x() - start.x(), targetPos.y() - start.y());

        for (double t = 0; t < maxTime; t += dt) {
            double vMag = Math.sqrt(vx * vx + vy * vy + vz * vz);

            // Calculate Drag Force coefficient
            double area = Math.PI * Math.pow(params.radius(), 2);
            double k = (0.5 * AIR_DENSITY * area * params.dragCoefficient()) / params.mass();

            // Apply accelerations (Drag + Gravity)
            double ax = -k * vMag * vx;
            double ay = -k * vMag * vy;
            double az = -GRAVITY - (k * vMag * vz);

            // Step position and velocity
            x += vx * dt; y += vy * dt; z += vz * dt;
            vx += ax * dt; vy += ay * dt; vz += az * dt;

            // Check if we have reached the target's horizontal distance
            if (Math.hypot(x - start.x(), y - start.y()) >= targetDistXY) {
                boolean hit = Math.abs(z - Constants.LauncherConstants.HUB_CENTER_HEIGHT) < 0.3;
                return new SimulationResult(hit, z - Constants.LauncherConstants.HUB_CENTER_HEIGHT, z);
            }

            if (z < 0) break;
        }
        return new SimulationResult(false, -100, 0);
    }

    /**
     * Translates the shooter's physical location from the Robot's frame to the Field's frame.
     * Essential because rotating the robot swings the shooter in a circle.
     */
    public static Vector2 getShooterFieldPosition(RobotState robot, double targetHeading) {
        double headingRad = Math.toRadians(targetHeading);
        
        // Define mechanical offsets in meters
        double lengthM = robot.length() * IN_TO_M; 
        double widthM = robot.width() * IN_TO_M;   
        double insetXM = (Constants.LauncherConstants.SHOOTER_SIZE_IN / 2.0) * IN_TO_M;
        
        // Coordinates relative to robot center (Facing +X)
        double localX = (-lengthM / 2.0) + insetXM; 
        double localY = (-widthM / 2.0) + (Constants.LauncherConstants.SHOOTER_SIDE_OFFSET_IN * IN_TO_M);
        
        // 2D Rotation Matrix to convert local offset to field offset
        double offsetX = localX * Math.cos(headingRad) - localY * Math.sin(headingRad);
        double offsetY = localX * Math.sin(headingRad) + localY * Math.cos(headingRad);
        
        return new Vector2(robot.x() + offsetX, robot.y() + offsetY);
    }

    /**
     * Solves for the required Turret Heading and Flywheel RPM.
     * Uses an iterative geometric solver for heading and binary search for velocity.
     */
    public static FiringSolution solveFiringSolution(RobotState robot, BallParams ball, Vector2 targetPos) {
        double robotSpeed = Math.hypot(robot.vx(), robot.vy());
        
        // Only handles stationary/slow shooting for this branch
        if (robotSpeed < 0.1) {
            double solvedHeading = robot.heading(); 
            
            // Iterate to converge on heading (compensates for shooter offset swinging)
            for (int i = 0; i < 8; i++) {
                Vector2 pos = getShooterFieldPosition(robot, solvedHeading);
                solvedHeading = Math.toDegrees(Math.atan2(targetPos.y() - pos.y(), targetPos.x() - pos.x()));
            }
            solvedHeading = (solvedHeading % 360 + 360) % 360;

            // Binary search to find the velocity that minimizes vertical error at the target
            Vector2 shooterPos = getShooterFieldPosition(robot, solvedHeading);
            double angleToTargetRad = Math.atan2(targetPos.y() - shooterPos.y(), targetPos.x() - shooterPos.x());
            double hoodRad = Math.toRadians(robot.hoodAngle());
            
            double minV = 5.0, maxV = 40.0, bestV = 0, minErr = 999;
            
            for (int i = 0; i < 12; i++) {
                double midV = (minV + maxV) / 2.0;
                
                // Break midV into 3D components
                double vH = midV * Math.cos(hoodRad);
                double vZ = midV * Math.sin(hoodRad);
                double vx = vH * Math.cos(angleToTargetRad);
                double vy = vH * Math.sin(angleToTargetRad);
                
                SimulationResult sim = simulateTrajectory(
                    new Vector3(shooterPos.x(), shooterPos.y(), robot.shooterHeight() * IN_TO_M),
                    new Velocity3(vx, vy, vZ),
                    targetPos, ball
                );
                
                double err = sim.heightAtTarget() - Constants.LauncherConstants.HUB_CENTER_HEIGHT;
                if (Math.abs(err) < Math.abs(minErr)) {
                    minErr = err;
                    bestV = midV;
                }
                
                if (err < 0) minV = midV; else maxV = midV;
            }

            // Convert meters/second to motor RPM
            double wheelRadiusM = ball.shooterWheelRadius() * IN_TO_M;
            double requiredRPM = bestV / ((2 * Math.PI / 60.0) * wheelRadiusM * ball.flywheelEfficiency());

            return new FiringSolution(requiredRPM, bestV, solvedHeading, Math.abs(minErr) < 0.5);
        }
        
        return new FiringSolution(0, 0, 0, false);
    }
}