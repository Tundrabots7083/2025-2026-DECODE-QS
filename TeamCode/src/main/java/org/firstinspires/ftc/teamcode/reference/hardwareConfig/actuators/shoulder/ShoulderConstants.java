package org.firstinspires.ftc.teamcode.reference.hardwareConfig.actuators.shoulder;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.MotorConstants;

public class ShoulderConstants {
        static {
                MotorConstants.name = "testMotor";//""shoulderMotor";
                MotorConstants.motorConfigurationType = "clone";
                MotorConstants.ticksPerRev=1397.1; // gobuilda ticks per rev for 60 rpm;
                MotorConstants.achievableMaxRPMFraction =1.0;
                MotorConstants.gearing=120.0 / 24.0;
                MotorConstants.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                MotorConstants.resetMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
                MotorConstants.direction = DcMotorSimple.Direction.FORWARD;
                MotorConstants.minPosition = -50.0;
                MotorConstants.maxPosition =90.0;
                MotorConstants.startPosition =-55.0;
                MotorConstants.targetPosition =-60.0;
                MotorConstants.feedforward=0.0;
                MotorConstants.tolerableError=0.7; //in degrees
                MotorConstants.kp=0.3;  //to be tuned for the PID controller
                MotorConstants.ki=0.01;
                MotorConstants.kd=0.02;

        }
}