package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.shooter;



import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ShooterTBHControllerConstantBase;

public class ShooterTBHControllerConstants {
    static {
        ShooterTBHControllerConstantBase.FRONT_Kp= 0.000008;  //to be tuned for the PID controller
        ShooterTBHControllerConstantBase.REAR_Kp= 0.000008;  //to be tuned for the PID controller
        ShooterTBHControllerConstantBase.motorMinPowerLimit= 0.0; // Always going forwards with a flywheel
        ShooterTBHControllerConstantBase.motorMaxPowerLimit= 1.0;
        ShooterTBHControllerConstantBase.FRONT_Kf_a = 0.0607; // a,b,c represent the values in
        ShooterTBHControllerConstantBase.FRONT_Kf_b = 1.42E-04; // a + bx + cx^2
        ShooterTBHControllerConstantBase.FRONT_Kf_c = 6.33E-09; // in the feedforward equation
        ShooterTBHControllerConstantBase.REAR_Kf_a = 0.0434;
        ShooterTBHControllerConstantBase.REAR_Kf_b = 1.5E-04;
        ShooterTBHControllerConstantBase.REAR_Kf_c = 4.64E-09;

    }
}