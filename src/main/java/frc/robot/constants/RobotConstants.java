package frc.robot.constants;

import frc.robot.constants.Constants;

public interface RobotConstants {

    int FRONT_LEFT_ENCODER_ID();
    int FRONT_RIGHT_ENCODER_ID();
    int BACK_LEFT_ENCODER_ID();
    int BACK_RIGHT_ENCODER_ID();

    double FRONT_LEFT_OFFSET();
    double FRONT_RIGHT_OFFSET();
    double BACK_LEFT_OFFSET();
    double BACK_RIGHT_OFFSET();

    boolean FRONT_LEFT_INVERTED();
    boolean FRONT_RIGHT_INVERTED();
    boolean BACK_LEFT_INVERTED();
    boolean BACK_RIGHT_INVERTED();

    int HOOD_ENCODER_ID();
    double HOOD_OFFSET();
    boolean HOOD_INVERTED();
}

