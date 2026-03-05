public class ShooterSolver {

    // --- TUNABLE CONSTANTS ---
    public static final double EFFICIENCY = 0.575; // Tunable efficiency factor
    public static final double HOOD_ANGLE_DEGREES = 60.0;
    public static final double SHOOTER_HEIGHT_METERS = 0.508; // ~20 inches
    public static final double WHEEL_RADIUS_METERS = 0.0508; // 2 inches
    
    // Robot Geometry (Meters)
    // Based on: Length 32in, Width 26in. Shooter at Back-Right.
    // Local offsets relative to robot center.
    // X is Front/Back, Y is Left/Right.
    // Shooter offset from center (Back edge + inset, Right edge + inset)
    public static final double SHOOTER_LOCAL_X = -0.36; // Approx -14.2 inches
    public static final double SHOOTER_LOCAL_Y = -0.25; // Approx -9.8 inches

    // Field / Target Constants
    public static final double GRAVITY = 9.81;
    public static final double TARGET_HEIGHT_METERS = 1.83; // 72 inches
    public static final double TARGET_RADIUS_METERS = 0.53; // ~21 inches
    public static final double TARGET_X = 4.62; // Blue Hub X
    public static final double TARGET_Y = 4.02; // Blue Hub Y

    // Ball Physics
    public static final double BALL_MASS_KG = 0.215;
    public static final double BALL_RADIUS_METERS = 0.075;
    public static final double DRAG_COEFF = 0.45;
    public static final boolean ENABLE_DRAG = true;

    // Solver Settings
    private static final double DT = 0.01;
    private static final double MAX_TIME = 4.0;

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
     * Main method to calculate firing solution.
     * @param robotX Robot center X on field (meters)
     * @param robotY Robot center Y on field (meters)
     * @return ShotResult containing required Heading and RPM
     */
    public static ShotResult solve(double robotX, double robotY) {
        // 1. Iteratively solve Heading (Shooter Offset Compensation)
        // We need the shooter to face the target, but the shooter is offset from the robot center.
        // As the robot turns, the shooter moves.
        double headingRad = 0.0;
        
        for (int i = 0; i < 8; i++) {
            Point shooterPos = calculateShooterPosition(robotX, robotY, headingRad);
            double dx = TARGET_X - shooterPos.x;
            double dy = TARGET_Y - shooterPos.y;
            headingRad = Math.atan2(dy, dx);
        }

        double headingDegrees = Math.toDegrees(headingRad);
        // Normalize 0-360
        headingDegrees = (headingDegrees % 360 + 360) % 360;

        // 2. Binary Search for Required Velocity
        Point finalShooterPos = calculateShooterPosition(robotX, robotY, headingRad);
        double dx = TARGET_X - finalShooterPos.x;
        double dy = TARGET_Y - finalShooterPos.y;
        double distanceToTarget = Math.hypot(dx, dy);
        
        double hoodRad = Math.toRadians(HOOD_ANGLE_DEGREES);
        
        double minV = 5.0;
        double maxV = 20.0;
        double bestV = 0.0;
        double minError = 999.0;

        for (int i = 0; i < 12; i++) {
            double v = (minV + maxV) / 2.0;
            
            // Decompose velocity vector
            double vZ = v * Math.sin(hoodRad);
            double vHorizontal = v * Math.cos(hoodRad);
            
            // Simulate
            SimResult sim = simulateShot(
                distanceToTarget, 
                SHOOTER_HEIGHT_METERS, 
                vHorizontal, 
                vZ
            );

            double error = sim.heightAtTarget - TARGET_HEIGHT_METERS;

            if (Math.abs(error) < Math.abs(minError)) {
                minError = error;
                bestV = v;
            }

            if (error < 0) {
                minV = v; // Shot too low, increase speed
            } else {
                maxV = v; // Shot too high, decrease speed
            }
        }

        // 3. Convert Velocity to RPM
        double rpm = calculateRPM(bestV);
        boolean found = Math.abs(minError) < 0.5; // Tolerance check

        return new ShotResult(headingDegrees, rpm, found);
    }

    /**
     * Simulates the ball flight using Euler integration.
     * Simplified to 2D (Distance/Height) since we aligned heading already.
     */
    private static SimResult simulateShot(double targetDist, double startZ, double vHoriz, double vZ) {
        double x = 0; // Distance traveled
        double z = startZ; // Height
        double vx = vHoriz;
        double vz_vel = vZ;

        double area = Math.PI * BALL_RADIUS_METERS * BALL_RADIUS_METERS;
        double k = (0.5 * 1.225 * area * DRAG_COEFF) / BALL_MASS_KG;

        for (double t = 0; t < MAX_TIME; t += DT) {
            double vMag = Math.sqrt(vx * vx + vz_vel * vz_vel);

            // Drag Forces
            double ax = 0;
            double az = -GRAVITY;

            if (ENABLE_DRAG) {
                ax = -k * vMag * vx;
                az -= (k * vMag * vz_vel);
            }

            // Integrate
            x += vx * DT;
            z += vz_vel * DT;
            vx += ax * DT;
            vz_vel += az * DT;

            // Check if we crossed the target distance
            if (x >= targetDist) {
                // Check if we hit the target height (simple cylinder check)
                return new SimResult(z);
            }

            if (z < 0) break; // Hit ground
        }

        return new SimResult(-100.0); // Missed
    }

    /**
     * Calculates absolute shooter position based on robot center and heading.
     */
    private static Point calculateShooterPosition(double robotX, double robotY, double headingRad) {
        // Rotate local offset
        // x_global = x_robot + (x_local * cos(theta) - y_local * sin(theta))
        // y_global = y_robot + (x_local * sin(theta) + y_local * cos(theta))
        
        double offsetX = SHOOTER_LOCAL_X * Math.cos(headingRad) - SHOOTER_LOCAL_Y * Math.sin(headingRad);
        double offsetY = SHOOTER_LOCAL_X * Math.sin(headingRad) + SHOOTER_LOCAL_Y * Math.cos(headingRad);

        return new Point(robotX + offsetX, robotY + offsetY);
    }

    private static double calculateRPM(double velocity) {
        // V_exit = V_tangential * Efficiency
        // V_tangential = V_exit / Efficiency
        // RPM = V_tangential / (2 * PI * r) * 60
        
        double vTangential = velocity / EFFICIENCY;
        double circumference = 2 * Math.PI * WHEEL_RADIUS_METERS;
        return (vTangential / circumference) * 60.0;
    }

    // --- Simple Helper Classes ---
    private static class Point {
        double x, y;
        Point(double x, double y) { this.x = x; this.y = y; }
    }

    private static class SimResult {
        double heightAtTarget;
        SimResult(double h) { this.heightAtTarget = h; }
    }
}