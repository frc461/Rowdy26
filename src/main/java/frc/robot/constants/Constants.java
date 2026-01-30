package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import java.util.function.Function;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {
    public static final class VisionConstants {
        public static Matrix<N3, N1> ODOM_STD_DEV;
        public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION;
        public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_FUNCTION;

        public static int PROXIMITY_SENSOR_DIO_PORT;

        public static final class LimelightConstants {
            public static String LIMELIGHT_NT_NAME;

            public static double LL_FORWARD;
            public static double LL_RIGHT;
            public static double LL_UP;
            public static double LL_ROLL;
            public static double LL_PITCH;
            public static double LL_YAW;

            public static double LL_MAX_TAG_CLEAR_DIST;
        }

        public static final class PhotonConstants {
            public static String BW_TOP_RIGHT_NAME;
            public static double BW_TOP_RIGHT_FORWARD;
            public static double BW_TOP_RIGHT_LEFT;
            public static double BW_TOP_RIGHT_UP;
            public static double BW_TOP_RIGHT_ROLL;
            public static double BW_TOP_RIGHT_PITCH;
            public static double BW_TOP_RIGHT_YAW;

            public static String BW_TOP_LEFT_NAME;
            public static double BW_TOP_LEFT_FORWARD;
            public static double BW_TOP_LEFT_LEFT;
            public static double BW_TOP_LEFT_UP;
            public static double BW_TOP_LEFT_ROLL;
            public static double BW_TOP_LEFT_PITCH;
            public static double BW_TOP_LEFT_YAW;

            public static String BW_BACK_NAME;
            public static double BW_BACK_FORWARD;
            public static double BW_BACK_LEFT;
            public static double BW_BACK_UP;
            public static double BW_BACK_ROLL;
            public static double BW_BACK_PITCH;
            public static double BW_BACK_YAW;

            public static double BW_MAX_TAG_CLEAR_DIST;

            public static double OBJECT_TARGET_PITCH;
        }
    }
}
