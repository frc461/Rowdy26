package frc.robot.constants.variants;

import frc.robot.constants.RobotConstants;

public class AlphaConstants implements RobotConstants {
    // CAN IDs
    public int FRONT_LEFT_ENCODER_ID() { return 21; }
    public int FRONT_RIGHT_ENCODER_ID() { return 24; }
    public int BACK_LEFT_ENCODER_ID() { return 22; }
    public int BACK_RIGHT_ENCODER_ID() { return 23; }

    // Offsets (you will tune these)
    public double FRONT_LEFT_OFFSET() { return 0.206298828125; }
    public double FRONT_RIGHT_OFFSET() { return -0.208740234375; }
    public double BACK_LEFT_OFFSET() { return 0.22436523475; }
    public double BACK_RIGHT_OFFSET() { return -0.34204105625; }

    // Inversion
    public boolean FRONT_LEFT_INVERTED() { return false; }
    public boolean FRONT_RIGHT_INVERTED() { return false; }
    public boolean BACK_LEFT_INVERTED() { return false; }
    public boolean BACK_RIGHT_INVERTED() { return false; }

    // Hood encoder
    public int HOOD_ENCODER_ID() { return 58; }
    public double HOOD_OFFSET() { return -0.865; }
    public boolean HOOD_INVERTED() { return true; }
}
