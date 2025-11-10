package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.carousel;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.PIDFControllerConstants;

public class CarouselPIDFControllerConstants {
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
