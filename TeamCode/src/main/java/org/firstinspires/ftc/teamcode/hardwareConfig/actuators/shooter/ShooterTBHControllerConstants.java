package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.shooter;



import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ShooterTBHControllerConstantBase;

public class ShooterTBHControllerConstants {
    static {
        ShooterTBHControllerConstantBase.FRONT_Kp= 0.000008;  //to be tuned for the PID controller
        ShooterTBHControllerConstantBase.FRONT_Kf=0.00015;
        ShooterTBHControllerConstantBase.REAR_Kp= 0.000008;  //to be tuned for the PID controller
        ShooterTBHControllerConstantBase.REAR_Kf=0.00015;
        ShooterTBHControllerConstantBase.motorMinPowerLimit= 0.0; // Always going forwards with a flywheel
        ShooterTBHControllerConstantBase.motorMaxPowerLimit= 1.0;

    }
}