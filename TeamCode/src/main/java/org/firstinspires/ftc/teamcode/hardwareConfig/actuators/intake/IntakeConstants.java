package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.MotorConstants;

/// TODO: set values for the arm motor
public class IntakeConstants {
    static {
        MotorConstants.name = "intakeMotor";
        MotorConstants.motorConfigurationType = "clone";
        MotorConstants.ticksPerRev=1397.1; // gobuilda ticks per rev for 60 rpm;
        MotorConstants.inchesPerRev=Math.PI* 1.4; //spool diameter
        MotorConstants.gearing=1.0;
        MotorConstants.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        MotorConstants.resetMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        MotorConstants.direction = DcMotorSimple.Direction.FORWARD;
        MotorConstants.minPosition = 0.3;
        MotorConstants.maxPosition =18;
        MotorConstants.startPosition =0.3;
        MotorConstants.targetPosition =0.3;//check if needed
        MotorConstants.feedforward=0.0;
        MotorConstants.tolerableError=0.2; //in inches
        MotorConstants.kp=0.3;  //to be tuned for the PID controller
        MotorConstants.ki=0.01;
        MotorConstants.kd=0.02;

    }
}