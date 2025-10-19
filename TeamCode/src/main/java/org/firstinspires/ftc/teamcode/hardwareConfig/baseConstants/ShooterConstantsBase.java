package org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ShooterConstantsBase extends MotorConstants{
    public static String frontMotorName;
    public static String rearMotorName;
    public static DcMotorSimple.Direction frontMotorDirection;
    public static DcMotorSimple.Direction rearMotorDirection;
    public static double FRONT_feedforward;
    public static double FRONT_Kp;
    public static double FRONT_Ki;
    public static double FRONT_Kd;
    public static double REAR_feedforward;
    public static double REAR_Kp;
    public static double REAR_Ki;
    public static double REAR_Kd;
}
