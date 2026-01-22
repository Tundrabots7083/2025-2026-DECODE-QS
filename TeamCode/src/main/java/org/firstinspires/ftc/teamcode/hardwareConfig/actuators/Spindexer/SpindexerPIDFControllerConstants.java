package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SpindexerPIDFControllerConstants {
    public static double kp = 0.015;
    public static double ki = 0.0015;
    public static double kd = 0.0003;
    public static double kf = 0;
    public static double maxIntegralSum = 0;
    public double motorMinPowerLimit = -1;
    public double motorMaxPowerLimit = 1;
}