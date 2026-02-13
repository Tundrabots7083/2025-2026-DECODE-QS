package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SpindexerPIDFControllerConstants {
    public static double kp = 0.035;
    public static double ki = 0.001;
    public static double kd = 0.0;
    public static double kf = 0.0006;
    public double maxIntegralSum = 100;
    public double maxVelocity = 600;
    public double maxAcceleration = 2000;
    public double motorMinPowerLimit = -0.5;
    public double motorMaxPowerLimit = 1;
}