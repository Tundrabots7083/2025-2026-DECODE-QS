package org.firstinspires.ftc.teamcode.opModes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.PIDFController;


@Autonomous(name="Sample Auto PIDF OpMode", group="specimens")
public class SampleAutoPIDFOpMode extends LinearOpMode
{

    private DcMotorEx motor;

    private PIDFController pidfController;

    public static double Kp=0.001;
    public static double Ki=0.0;
    public static double Kd=0.0;
    public static double Kf=0.0;

    public static double motorPower=0.1;
    public static double targetPosition = -3000.0;

    private long count=0;


    @Override
    public void runOpMode()
    {

        telemetry.addData("TestMotorOpMode", "runOpMode started");
        telemetry.update();
        waitForStart();

        motor = hardwareMap.get(DcMotorEx.class, "testMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize PIDF controller with example gains (tune these!)

        pidfController = new PIDFController(Kp, Ki, Kd, Kf);
        pidfController.setOutputLimits(-1.0, 1.0); // Motor power limits
        pidfController.setMaxIntegralSum(100.0); // Prevent integral windup

        // Example target position (e.g., encoder ticks)

        while (opModeIsActive())
        {
            count++;
            telemetry.addData("TestMotorOpMode", "runOpMode while started count: %d",count);
            telemetry.update();

            double currentPosition = motor.getCurrentPosition();

            //// Calculate motor power using PIDF
            double power = pidfController.calculate(targetPosition, currentPosition);

            // Apply power to motor
            motor.setPower(power);
            //motor.setPower(motorPower);

            // Telemetry for debugging
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Power", power);
            //joinedTelemetry.addData("Power", motorPower);
            telemetry.update();

        }
    }

}
