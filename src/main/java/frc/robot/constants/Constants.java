package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class Constants {


    public final class VisionConstants {

        public static final NetworkTableInstance NT_INSTANCE = NetworkTableInstance.getDefault();

        public static final String CAMERA_FR_NAME = "CAMERA_FR"; 
        public static final double CAMERA_FR_YAW = 0;
        public static final double CAMERA_FR_PITCH = 28.812;
        public static final double CAMERA_FR_ROLL = 0;
        public static final double CAMERA_FR_FORWARD = Units.inchesToMeters(15.03);
        public static final double CAMERA_FR_LEFT = Units.inchesToMeters(-1.475);
        public static final double CAMERA_FR_UP = Units.inchesToMeters(20);

        public static final String CAMERA_FL_NAME = "CAMERA_FL"; 
        public static final double CAMERA_FL_YAW = 45;
        public static final double CAMERA_FL_PITCH = 14.864;
        public static final double CAMERA_FL_ROLL = 0;
        public static final double CAMERA_FL_FORWARD = Units.inchesToMeters(9.893);
        public static final double CAMERA_FL_LEFT = Units.inchesToMeters(-7.310);
        public static final double CAMERA_FL_UP = Units.inchesToMeters(20);

        public static final String CAMERA_BR_NAME = "CAMERA_BR"; 
        public static final double CAMERA_BR_YAW = 0;
        public static final double CAMERA_BR_PITCH = 0;
        public static final double CAMERA_BR_ROLL = 0;
        public static final double CAMERA_BR_FORWARD = 0;
        public static final double CAMERA_BR_LEFT = 0;
        public static final double CAMERA_BR_UP = 0;


        
        // Trust Constants

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final double MAX_VALID_DIST = 5.0; //Meters
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static final Matrix<N3, N1> ODOM_STD_DEV = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(0.01));


    }

    public final class LauncherConstants {
        public static double SHOOTER_SIZE_IN = 7.0;
    
    /** Center of shooter is 11.5 inches from the side edge */
        public static double SHOOTER_SIDE_OFFSET_IN = 11.5;

        public static double HUB_CENTER_HEIGHT = 72;

        public static double HUB_HOOD_ANGLE = 0.0;
        public static double HUB_RPM = -1950.0;

        public static double TOWER_HOOD_ANGLE = 1.25;
        public static double TOWER_RPM = -2250.0;
    }
}
