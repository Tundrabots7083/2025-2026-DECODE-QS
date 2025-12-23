package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.intake;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.VelocityTBHControllerConstantBase;


public class IntakeTBHControllerConstants {
    static {
        VelocityTBHControllerConstantBase.FRONT_Kp= 0.08;  //to be tuned for the PID controller
        VelocityTBHControllerConstantBase.FRONT_Kf_a = 0.837; // a,b,c represent the values in
        VelocityTBHControllerConstantBase.FRONT_Kf_b = 1.42E-04; // a + bx + cx^2
        VelocityTBHControllerConstantBase.FRONT_Kf_c = 0; // in the feedforward equation
    }
}