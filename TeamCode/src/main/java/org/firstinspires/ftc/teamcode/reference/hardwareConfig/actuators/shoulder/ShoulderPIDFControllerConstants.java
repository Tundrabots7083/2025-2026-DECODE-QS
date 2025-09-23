package org.firstinspires.ftc.teamcode.reference.hardwareConfig.actuators.shoulder;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.PIDFControllerConstants;


public class ShoulderPIDFControllerConstants {
    static {
        PIDFControllerConstants.kp=0.01;  //to be tuned for the PID controller
        PIDFControllerConstants.ki=0.0001;
        PIDFControllerConstants.kd=0.0;
        PIDFControllerConstants.kf=0.0;
        PIDFControllerConstants.maxIntegralSum=100;
        PIDFControllerConstants.motorMinPowerLimit=-1;
        PIDFControllerConstants.motorMaxPowerLimit=1;

    }
}