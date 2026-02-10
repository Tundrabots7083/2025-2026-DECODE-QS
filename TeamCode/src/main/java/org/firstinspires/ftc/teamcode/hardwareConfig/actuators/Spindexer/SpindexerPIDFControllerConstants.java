package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SpindexerPIDFControllerConstants {
    public static double kp = 0.01;
    public static double ki = 0.001;
    public static double kd = 0.0;
    public static double kf = 0.0008;
    public double maxIntegralSum = 100;
    public double maxVelocity = 1500;
    public double maxAcceleration = 5000;
    public double motorMinPowerLimit = -0.5;
    public double motorMaxPowerLimit = 1;
}