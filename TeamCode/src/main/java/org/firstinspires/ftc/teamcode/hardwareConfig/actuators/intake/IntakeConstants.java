package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.VelocityMotorConstantsBase;

/// TODO: set values for the arm motor
public class IntakeConstants {
    static {
        VelocityMotorConstantsBase.frontMotorName = "rightRear";
        VelocityMotorConstantsBase.motorConfigurationType = "clone";
        VelocityMotorConstantsBase.ticksPerRev= 28; // gobilda ticks 6000 rpm
        VelocityMotorConstantsBase.achievableMaxRPMFraction =1.0;
        VelocityMotorConstantsBase.gearing=1;
        VelocityMotorConstantsBase.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        VelocityMotorConstantsBase.resetMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        VelocityMotorConstantsBase.frontMotorDirection = DcMotorSimple.Direction.REVERSE;
        VelocityMotorConstantsBase.rearMotorDirection = DcMotorSimple.Direction.FORWARD;
        VelocityMotorConstantsBase.startPosition =0;
        VelocityMotorConstantsBase.targetPosition =0;
        VelocityMotorConstantsBase.tolerableError=0.7; //in degrees
    }
}