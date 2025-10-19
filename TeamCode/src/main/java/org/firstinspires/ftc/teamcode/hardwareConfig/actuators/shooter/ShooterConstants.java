package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.shooter;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ShooterConstantsBase;

public class ShooterConstants {
        static {
                ShooterConstantsBase.frontMotorName = "rightRear";
                ShooterConstantsBase.rearMotorName = "frontShooterMotor";
                ShooterConstantsBase.motorConfigurationType = "clone";
                ShooterConstantsBase.ticksPerRev= 1993.6; // gobilda ticks 84 rpm// 537.7; // gobilda ticks per rev for 312 rpm //1397.1; // gobuilda ticks per rev for 60 rpm;
                ShooterConstantsBase.achievableMaxRPMFraction =1.0;
                ShooterConstantsBase.gearing=1;
                ShooterConstantsBase.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                ShooterConstantsBase.resetMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
                ShooterConstantsBase.frontMotorDirection = DcMotorSimple.Direction.REVERSE;
                ShooterConstantsBase.rearMotorDirection = DcMotorSimple.Direction.REVERSE;
                ShooterConstantsBase.startPosition =0;
                ShooterConstantsBase.targetPosition =0;
                ShooterConstantsBase.tolerableError=0.7; //in degrees
                ShooterConstantsBase.FRONT_Kp=0.3;  //to be tuned for the PID controller
                ShooterConstantsBase.FRONT_feedforward=0.3;
                ShooterConstantsBase.REAR_Kp=0.3;
                ShooterConstantsBase.REAR_feedforward=0.3;


        }
}