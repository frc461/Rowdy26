package frc.robot;

import java.net.NetworkInterface;
import java.util.Collections;

import frc.robot.constants.Constants.RobotType;

public class RobotIdentifier {

    public static RobotType getRobot() {
        try {
            for (NetworkInterface ni : Collections.list(NetworkInterface.getNetworkInterfaces())) {

                byte[] mac = ni.getHardwareAddress();
                if (mac == null) continue;

                StringBuilder macString = new StringBuilder();
                for (byte b : mac) {
                    macString.append(String.format("%02X:", b));
                }

                String address = macString.toString();

                // the comp mac address still needs to be found
                if (address.startsWith("00:80:2F:AA:BB:CC")) {
                    return RobotType.COMP;
                }
                if (address.startsWith("00:80:2F:34:07:F0")) {
                    return RobotType.ALPHA;
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return RobotType.UNKNOWN;
    }
}