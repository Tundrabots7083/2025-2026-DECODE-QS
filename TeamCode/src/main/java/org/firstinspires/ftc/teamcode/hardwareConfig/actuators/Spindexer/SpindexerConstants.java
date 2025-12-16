package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.MotorConstants;

public class SpindexerConstants {
    static{
        MotorConstants.name = "Spindex Motor";
        MotorConstants.motorConfigurationType = "clone";
        MotorConstants.ticksPerRev = 435; //placeholder
        MotorConstants.gearing = 5;
        MotorConstants.achievableMaxRPMFraction = 1;
        MotorConstants.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        MotorConstants.resetMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        MotorConstants.direction = DcMotorSimple.Direction.FORWARD;
        MotorConstants.startPosition = 0.0;
        MotorConstants.tolerableError = 1.0;
    }
}
