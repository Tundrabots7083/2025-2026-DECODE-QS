package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.shooter;



import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.VelocityTBHControllerConstantBase;

public class ShooterTBHControllerConstants {
    static {
        VelocityTBHControllerConstantBase.FRONT_Kp= 0.000008;  //to be tuned for the PID controller
        VelocityTBHControllerConstantBase.REAR_Kp= 0.000008;  //to be tuned for the PID controller
        VelocityTBHControllerConstantBase.FRONT_Kf_a = 0.0607; // a,b,c represent the values in
        VelocityTBHControllerConstantBase.FRONT_Kf_b = 1.42E-04; // a + bx + cx^2
        VelocityTBHControllerConstantBase.FRONT_Kf_c = 6.33E-09; // in the feedforward equation
        VelocityTBHControllerConstantBase.REAR_Kf_a = 0.0434;
        VelocityTBHControllerConstantBase.REAR_Kf_b = 1.5E-04;
        VelocityTBHControllerConstantBase.REAR_Kf_c = 4.64E-09;

    }
}