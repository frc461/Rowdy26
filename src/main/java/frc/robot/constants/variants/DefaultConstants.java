package frc.robot.constants.variants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class DefaultConstants {


    public static final class VisionConstants {
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
        public static final double CAMERA_BR_YAW = 0;
        public static final double CAMERA_BR_PITCH = 0;
        public static final double CAMERA_BR_ROLL = 0;
        public static final double CAMERA_BR_FORWARD = 0;
        public static final double CAMERA_BR_LEFT = 0;
        public static final double CAMERA_BR_UP = 0;
    }
}
