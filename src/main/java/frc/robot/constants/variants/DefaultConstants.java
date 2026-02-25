package frc.robot.constants.variants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class DefaultConstants {

    public static final class VisionConstants {
        public static final String CAMERA_FR_NAME = "CAMERA_FR"; 
        public static final double CAMERA_FR_YAW = 0;
        public static final double CAMERA_FR_PITCH = 28.812;
        public static final double CAMERA_FR_ROLL = 0;
        public static final double CAMERA_FR_FORWARD = 15.03;
        public static final double CAMERA_FR_LEFT = -1.475;
        public static final double CAMERA_FR_UP = 20;

        public static final String CAMERA_FL_NAME = "CAMERA_FL"; 
        public static final double CAMERA_FL_YAW = 45;
        public static final double CAMERA_FL_PITCH = 14.864;
        public static final double CAMERA_FL_ROLL = 0;
        public static final double CAMERA_FL_FORWARD = 9.893;
        public static final double CAMERA_FL_LEFT = -7.310;
        public static final double CAMERA_FL_UP = 20;

        public static final String CAMERA_BR_NAME = "CAMERA_BR"; 
        public static final double CAMERA_BR_YAW = 0;
        public static final double CAMERA_BR_PITCH = 0;
        public static final double CAMERA_BR_ROLL = 0;
        public static final double CAMERA_BR_FORWARD = 0;
        public static final double CAMERA_BR_LEFT = 0;
        public static final double CAMERA_BR_UP = 0;


        //TWEAK VALUES IN TESTING
        public static double MAX_VALID_DIST = 5.0; //Meters
        public static Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static Matrix<N3, N1> ODOM_STD_DEV = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(0.01));
    }

    public static final class SwerveConstants {

    }





}
