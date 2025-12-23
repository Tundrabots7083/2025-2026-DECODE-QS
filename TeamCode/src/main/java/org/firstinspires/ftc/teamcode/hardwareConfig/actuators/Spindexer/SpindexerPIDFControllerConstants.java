package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.PIDFControllerConstants;

public class SpindexerPIDFControllerConstants {
    static {
        PIDFControllerConstants.kp = 0.015;
        PIDFControllerConstants.ki = 0.0015;
        PIDFControllerConstants.kd = 0.0003;
        PIDFControllerConstants.kf = 0;
        PIDFControllerConstants.maxIntegralSum = 0;
        PIDFControllerConstants.motorMinPowerLimit = 0;
        PIDFControllerConstants.motorMaxPowerLimit = 0;
    }
}
