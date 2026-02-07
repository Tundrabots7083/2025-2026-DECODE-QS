package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SpindexerConstants {
    public String name = "Spindexer";
    public String motorConfigurationType = "clone";
    public double ticksPerRev = 384.5; //placeholder
    public double gearing = 1 / 1.5;
    public double achievableMaxRPMFraction = 1;
    public DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public DcMotor.RunMode resetMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
    public DcMotorSimple.Direction direction = DcMotorSimple.Direction.REVERSE;
    public double startPosition = 0.0;
    public double tolerableError = 5.5;
    public double tolerableVelocityError = 20;
    public double degreeOffset = 30.0;
}
