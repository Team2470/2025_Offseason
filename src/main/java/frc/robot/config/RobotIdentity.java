package frc.robot.config;

import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.MacAddressUtil;

import static frc.robot.util.MacAddressUtil.getMACAddress;

public enum RobotIdentity {
    KRILLION,
    SIMULATION,
    ;

    public static RobotIdentity getIdentity() {
        if (!Robot.isReal()) {
            return SIMULATION;
        } else {
            String mac = getMACAddress();
            if (!mac.isEmpty()) {
                if (mac.equals(MacAddressUtil.Krillion)) {
                    return KRILLION;
                }
            }

            return KRILLION;
        }
    }

    public static Constants.Mode getMode() {
        return switch (getIdentity()) {
            case KRILLION -> Robot.isReal() ? Constants.Mode.REAL : Constants.Mode.REPLAY;
            case SIMULATION -> Constants.Mode.SIM;
        };
    }
}
