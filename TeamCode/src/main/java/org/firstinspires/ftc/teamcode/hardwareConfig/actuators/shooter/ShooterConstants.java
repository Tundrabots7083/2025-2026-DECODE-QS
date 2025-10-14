package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.shooter;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.MotorConstants;

public class ShooterConstants {
        static {
                MotorConstants.name = "rightRear";
                MotorConstants.motorConfigurationType = "clone";
                MotorConstants.ticksPerRev= 1993.6; // gobilda ticks 84 rpm// 537.7; // gobilda ticks per rev for 312 rpm //1397.1; // gobuilda ticks per rev for 60 rpm;
                MotorConstants.achievableMaxRPMFraction =1.0;
                MotorConstants.gearing=1;
                MotorConstants.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                MotorConstants.resetMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
                MotorConstants.direction = DcMotorSimple.Direction.REVERSE;
                MotorConstants.startPosition =0;
                MotorConstants.targetPosition =0;
                MotorConstants.feedforward=0.0;
                MotorConstants.tolerableError=0.7; //in degrees
                MotorConstants.kp=0.3;  //to be tuned for the PID controller
                MotorConstants.ki=0;
                MotorConstants.kd=0;

        }
}