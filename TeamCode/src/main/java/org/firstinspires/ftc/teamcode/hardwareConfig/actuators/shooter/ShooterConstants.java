package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.shooter;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ShooterConstantsBase;

public class ShooterConstants {
        static {
                ShooterConstantsBase.frontMotorName = "rightRear";
                ShooterConstantsBase.rearMotorName = "leftRear";
                ShooterConstantsBase.motorConfigurationType = "clone";
                ShooterConstantsBase.ticksPerRev= 28; // gobilda ticks 6000 rpm
                ShooterConstantsBase.achievableMaxRPMFraction =1.0;
                ShooterConstantsBase.gearing=1;
                ShooterConstantsBase.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                ShooterConstantsBase.resetMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
                ShooterConstantsBase.frontMotorDirection = DcMotorSimple.Direction.REVERSE;
                ShooterConstantsBase.rearMotorDirection = DcMotorSimple.Direction.FORWARD;
                ShooterConstantsBase.startPosition =0;
                ShooterConstantsBase.targetPosition =0;
                ShooterConstantsBase.tolerableError=0.7; //in degrees
        }
}