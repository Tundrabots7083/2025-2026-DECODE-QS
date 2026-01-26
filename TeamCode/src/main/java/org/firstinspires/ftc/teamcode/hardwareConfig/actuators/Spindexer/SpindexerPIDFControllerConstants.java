package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SpindexerPIDFControllerConstants {
    public static double kp = 0.05;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.00075;
    public double maxIntegralSum = 100;
    public static double maxVelocity = 420;
    public static double maxAcceleration = 2000;
    public double motorMinPowerLimit = -1;
    public double motorMaxPowerLimit = 1;
}