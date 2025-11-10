package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.shooter;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.PIDFControllerConstants;


public class ShooterTBHControllerConstants {
    static {
        PIDFControllerConstants.kp=0.01;  //to be tuned for the PID controller
        PIDFControllerConstants.kf=0.0;
        PIDFControllerConstants.motorMinPowerLimit= 0.0; // Always going forwards with a flywheel
        PIDFControllerConstants.motorMaxPowerLimit= 1.0;

    }
}