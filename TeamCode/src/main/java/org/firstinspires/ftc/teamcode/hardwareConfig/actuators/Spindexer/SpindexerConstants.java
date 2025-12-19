package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.MotorConstants;

public class SpindexerConstants {
    static{
        MotorConstants.name = "leftFront";
        MotorConstants.motorConfigurationType = "clone";
        MotorConstants.ticksPerRev = 384.5; //placeholder
        MotorConstants.gearing = 1;
        MotorConstants.achievableMaxRPMFraction = 1;
        MotorConstants.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        MotorConstants.resetMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        MotorConstants.direction = DcMotorSimple.Direction.FORWARD;
        MotorConstants.startPosition = 0.0;
        MotorConstants.tolerableError = 0.5;
    }
}
