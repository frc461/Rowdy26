package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotIdentifier;
import frc.robot.constants.variants.AlphaConstants;
import frc.robot.constants.variants.CompConstants;

public final class Constants implements RobotConstants{

    public static NetworkTableInstance NT_INSTANCE;

    public enum RobotType {
    COMP,
    ALPHA,
    UNKNOWN
    }

    public static final RobotConstants CURRENT;

    static {
        switch (RobotIdentifier.getRobot()) {
            case COMP:
                CURRENT = new CompConstants();
                break;
            case ALPHA:
                CURRENT = new AlphaConstants();
                break;
            default:
                CURRENT = new Constants();
                break;
        }
    }

    public static final RobotType ROBOT = RobotIdentifier.getRobot();

    public final class SwerveConstants {
        public static Translation2d k_frontLeftLocation;
        public static Translation2d k_frontRightLocation;
        public static Translation2d k_backLeftLocation;
        public static Translation2d k_backRightLocation;
    }

    

    public final class LauncherConstants {
        public static double SHOOTER_SIZE_IN = 7.0;
    
    /** Center of shooter is 11.5 inches from the side edge */
        public static double SHOOTER_SIDE_OFFSET_IN = 11.5;

        public static double HUB_CENTER_HEIGHT = 72;

        public static double HUB_HOOD_ANGLE = 0.0;
        public static double HUB_RPM = -1725.0;

        public static double TOWER_HOOD_ANGLE = 1.25;
        public static double TOWER_RPM = -2125.0;

        public static double TEMP_AUTO_START_HOOD_ANGLE = 1.25;
        public static double TEMP_AUTO_RPM = -2450.0;

        public static double TRENCH_AUTO_START_HOOD_ANGLE = 1.25;
        public static double TRENCH_AUTO_RPM = -2375.0;

        public static double ABSOLUTE_ENCODER_OFFSET = -0.865;
        public static double ENCODER_CONVERSION = 3.942;


        public static double ROTOR_TO_SENSOR_RATIO = 4.0;
    }



    @Override
    public int FRONT_LEFT_ENCODER_ID() {
         throw new UnsupportedOperationException("Unimplemented method 'FRONT_LEFT_ENCODER_ID'");
    }



    @Override
    public int FRONT_RIGHT_ENCODER_ID() {
         throw new UnsupportedOperationException("Unimplemented method 'FRONT_RIGHT_ENCODER_ID'");
    }



    @Override
    public int BACK_LEFT_ENCODER_ID() {
        throw new UnsupportedOperationException("Unimplemented method 'BACK_LEFT_ENCODER_ID'");
    }



    @Override
    public int BACK_RIGHT_ENCODER_ID() {
         throw new UnsupportedOperationException("Unimplemented method 'BACK_RIGHT_ENCODER_ID'");
    }



    @Override
    public double FRONT_LEFT_OFFSET() {
         throw new UnsupportedOperationException("Unimplemented method 'FRONT_LEFT_OFFSET'");
    }



    @Override
    public double FRONT_RIGHT_OFFSET() {
         throw new UnsupportedOperationException("Unimplemented method 'FRONT_RIGHT_OFFSET'");
    }



    @Override
    public double BACK_LEFT_OFFSET() {
         throw new UnsupportedOperationException("Unimplemented method 'BACK_LEFT_OFFSET'");
    }



    @Override
    public double BACK_RIGHT_OFFSET() {
         throw new UnsupportedOperationException("Unimplemented method 'BACK_RIGHT_OFFSET'");
    }



    @Override
    public boolean FRONT_LEFT_INVERTED() {
         throw new UnsupportedOperationException("Unimplemented method 'FRONT_LEFT_INVERTED'");
    }



    @Override
    public boolean FRONT_RIGHT_INVERTED() {
         throw new UnsupportedOperationException("Unimplemented method 'FRONT_RIGHT_INVERTED'");
    }



    @Override
    public boolean BACK_LEFT_INVERTED() {
         throw new UnsupportedOperationException("Unimplemented method 'BACK_LEFT_INVERTED'");
    }



    @Override
    public boolean BACK_RIGHT_INVERTED() {
         throw new UnsupportedOperationException("Unimplemented method 'BACK_RIGHT_INVERTED'");
    }



    @Override
    public int HOOD_ENCODER_ID() {
         throw new UnsupportedOperationException("Unimplemented method 'HOOD_ENCODER_ID'");
    }



    @Override
    public double HOOD_OFFSET() {
         throw new UnsupportedOperationException("Unimplemented method 'HOOD_OFFSET'");
    }



    @Override
    public boolean HOOD_INVERTED() {
         throw new UnsupportedOperationException("Unimplemented method 'HOOD_INVERTED'");
    }
}