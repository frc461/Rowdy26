package frc.robot.constants;

import edu.wpi.first.networktables.NetworkTableInstance;

public final class Constants {

    public static NetworkTableInstance NT_INSTANCE;

    public final class VisionConstants {

        public static String CAMERA_FR_NAME;
        public static double CAMERA_FR_YAW;
        public static double CAMERA_FR_PITCH;
        public static double CAMERA_FR_ROLL;
        public static double CAMERA_FR_FORWARD;
        public static double CAMERA_FR_LEFT;
        public static double CAMERA_FR_UP;

        public static String CAMERA_FL_NAME;
        public static double CAMERA_FL_YAW;
        public static double CAMERA_FL_PITCH;
        public static double CAMERA_FL_ROLL;
        public static double CAMERA_FL_FORWARD;
        public static double CAMERA_FL_LEFT;
        public static double CAMERA_FL_UP;

        public static String CAMERA_BR_NAME; 
        public static double CAMERA_BR_YAW;
        public static double CAMERA_BR_PITCH;
        public static double CAMERA_BR_ROLL;
        public static double CAMERA_BR_FORWARD;
        public static double CAMERA_BR_LEFT;
        public static double CAMERA_BR_UP;

        public static String CAMERA_BL_NAME; 
        public static double CAMERA_BL_YAW;
        public static double CAMERA_BL_PITCH;
        public static double CAMERA_BL_ROLL;
        public static double CAMERA_BL_FORWARD;
        public static double CAMERA_BL_LEFT;
        public static double CAMERA_BL_UP;


    }

    public final class LauncherConstants {
        public static double SHOOTER_SIZE_IN = 7.0;
    
    /** Center of shooter is 11.5 inches from the side edge */
        public static double SHOOTER_SIDE_OFFSET_IN = 11.5;

        public static double HUB_CENTER_HEIGHT = 72;
    }
}