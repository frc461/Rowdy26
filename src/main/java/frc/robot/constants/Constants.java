package frc.robot.constants;

import edu.wpi.first.networktables.NetworkTableInstance;

public final class Constants {

    public static NetworkTableInstance NT_INSTANCE;

    public final class VisionConstants {

        public static String CAMERA_RED_NAME;
        public static double CAMERA_RED_YAW;
        public static double CAMERA_RED_PITCH;
        public static double CAMERA_RED_ROLL;
        public static double CAMERA_RED_FORWARD;
        public static double CAMERA_RED_LEFT;
        public static double CAMERA_RED_UP;

        public static String CAMERA_GRAY_NAME;
        public static double CAMERA_GRAY_YAW;
        public static double CAMERA_GRAY_PITCH;
        public static double CAMERA_GRAY_ROLL;
        public static double CAMERA_GRAY_FORWARD;
        public static double CAMERA_GRAY_LEFT;
        public static double CAMERA_GRAY_UP;


    }

    public final class LauncherConstants {
        public static double SHOOTER_SIZE_IN = 7.0;
    
    /** Center of shooter is 11.5 inches from the side edge */
        public static double SHOOTER_SIDE_OFFSET_IN = 11.5;

        public static double HUB_CENTER_HEIGHT = 72;
    }
}