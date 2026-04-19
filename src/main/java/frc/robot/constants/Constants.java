package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class Constants {

    public static NetworkTableInstance NT_INSTANCE;

    public final class VisionConstants {
        // Trust Constants

        // --- STANDARD DEVIATIONS (Trust Matrices) ---
        // The values are [X, Y, Theta]. Lower numbers mean you trust the camera MORE.
        // Higher numbers mean you trust the camera LESS.
        
        // When you only see 1 tag, trust the camera less (especially rotation)
        public static final Matrix<N3, N1> kSINGLE_TAG_STD_DEVS = VecBuilder.fill(0.9, 0.9, 0.9);
        
        // When you see 2+ tags, PhotonVision's PNP math is highly accurate. Trust it more.
        public static final Matrix<N3, N1> kMULTI_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 0.1);


        // --- CAMERA 1: FRONT RIGHT (FR) ---
        // MUST match exactly what you typed in the PhotonVision Web UI
        public static final String CAMERA_FR_NAME = "CAMERA_FR"; 
        public static final double CAMERA_FR_YAW = -5.0;
        public static final double CAMERA_FR_PITCH = 28.812;
        public static final double CAMERA_FR_ROLL = 0;
        public static final double CAMERA_FR_FORWARD = Units.inchesToMeters(15.03);
        public static final double CAMERA_FR_LEFT = Units.inchesToMeters(-1.475);
        public static final double CAMERA_FR_UP = Units.inchesToMeters(20);

        public static final String CAMERA_FL_NAME = "CAMERA_FL"; 
        public static final double CAMERA_FL_YAW = -135;
        public static final double CAMERA_FL_PITCH = 14.864;
        public static final double CAMERA_FL_ROLL = 0;
        public static final double CAMERA_FL_FORWARD = Units.inchesToMeters(7.893);
        public static final double CAMERA_FL_LEFT = Units.inchesToMeters(-7.310);
        public static final double CAMERA_FL_UP = Units.inchesToMeters(20);

         public static final String CAMERA_BR_NAME = "CAMERA_BR"; 
        public static final double CAMERA_BR_YAW = 180.0;
        public static final double CAMERA_BR_PITCH = 15.0;
        public static final double CAMERA_BR_ROLL = 0.0;
        public static final double CAMERA_BR_FORWARD = Units.inchesToMeters(-11.379);   
        public static final double CAMERA_BR_LEFT = Units.inchesToMeters(-9.25);
        public static final double CAMERA_BR_UP = Units.inchesToMeters(8.51);
    }

    public final class SwerveConstants {
        public static Translation2d k_frontLeftLocation;
        public static Translation2d k_frontRightLocation;
        public static Translation2d k_backLeftLocation;
        public static Translation2d k_backRightLocation;
    }

    

    public final class LauncherConstants {

        public static double LAUNCHER_MULT = 1.40;
        public static double SHOOTER_SIZE_IN = 7.0;
    
    /** Center of shooter is 11.5 inches from the side edge */
        public static double SHOOTER_SIDE_OFFSET_IN = 11.5;

        public static double HUB_CENTER_HEIGHT = 72;

        public static double HUB_HOOD_ANGLE = 0.0;
        public static double HUB_RPM = -1600.0 * LAUNCHER_MULT;

        public static double TOWER_HOOD_ANGLE = 1.15;
        public static double TOWER_RPM = -2300.0 * LAUNCHER_MULT;

        public static double TEMP_AUTO_START_HOOD_ANGLE = 1.20;
        public static double TEMP_AUTO_RPM = -2450.0 * LAUNCHER_MULT;

        public static double TRENCH_AUTO_START_HOOD_ANGLE = 1.25;
        public static double TRENCH_AUTO_RPM = -2250.0 * LAUNCHER_MULT;

        public static double SHUTTLE_AUTO_START_HOOD_ANGLE = 2.30;
        public static double SHUTTLE_AUTO_RPM = -3300.0 * LAUNCHER_MULT;

        public static double ABSOLUTE_ENCODER_OFFSET = 0.23;
        public static double ENCODER_CONVERSION = 3.942;


        public static double ROTOR_TO_SENSOR_RATIO = 4.0;

        public static final double SEVENTY_DEG_HOOD_ANGLE = 0.15; //Close
        public static final double SIXTY_DEG_HOOD_ANGLE = 1.15; //Far
        public static final double FIFTY_DEG_HOOD_ANGLE = 1.567; //Really Far


        public static double AUTO_AIM_HOOD_ANGLE = 0.0;
        public static final double ABSOLUTE_ENCODER_DIS_PT = 0.1;
        
    }
}