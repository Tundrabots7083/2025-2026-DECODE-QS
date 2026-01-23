package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.shooter;


public class ShooterTBHControllerConstants {
    public double FRONT_Kp = 0.000008;  //to be tuned for the PID controller
    public double REAR_Kp = 0.000008;  //to be tuned for the PID controller
    public double FRONT_Kf_a = 0.0827; // a,b,c represent the values in
    public double FRONT_Kf_b = 1.15E-04; // a + bx + cx^2
    public double FRONT_Kf_c = 1.03E-08; // in the feedforward equation
    public double REAR_Kf_a = 0.0434;
    public double REAR_Kf_b = 1.5E-04;
    public double REAR_Kf_c = 4.64E-09;

}