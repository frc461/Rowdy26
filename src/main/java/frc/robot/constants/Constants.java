package frc.robot.constants;

import edu.wpi.first.math.Matrix;

import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public final class Constants {

    public static final NetworkTableInstance NT_INSTANCE = null;
    public static Supplier<DriverStation.Alliance> ALLIANCE_SUPPLIER;

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

        public static NetworkTableInstance NT_INSTANCE;

        public static final class PhotonConstants {
            public static String COLOR_NAME;
            public static double COLOR_FORWARD;
            public static double COLOR_LEFT;
            public static double COLOR_UP;
            public static double COLOR_ROLL;
            public static double COLOR_PITCH;
            public static double COLOR_YAW;

            public static String BW_NAME;
            public static double BW_FORWARD;
            public static double BW_LEFT;
            public static double BW_UP;
            public static double BW_ROLL;
            public static double BW_PITCH;
            public static double BW_YAW;

            public static String BW_FRONT_NAME;
            public static double BW_FRONT_FORWARD;
            public static double BW_FRONT_LEFT;
            public static double BW_FRONT_UP;
            public static double BW_FRONT_ROLL;
            public static double BW_FRONT_PITCH;
            public static double BW_FRONT_YAW;

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
