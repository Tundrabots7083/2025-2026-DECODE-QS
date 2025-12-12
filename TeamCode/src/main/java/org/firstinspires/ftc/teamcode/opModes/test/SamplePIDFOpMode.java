package org.firstinspires.ftc.teamcode.opModes.test;

//import com.pedropathing.util.PIDFController;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.PIDFController;

@TeleOp(name="Sample PIDF OpMode")
public class SamplePIDFOpMode extends LinearOpMode {
    private DcMotor motor;
    private PIDFController pidfController;

    @Override
    public void runOpMode() {
        // Initialize motor
        motor = hardwareMap.get(DcMotor.class, "testMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorEx.Direction.FORWARD);

        // Initialize PIDF controller with example gains (tune these!)
        pidfController = new PIDFController(0.01, 0.0001, 0.1, 0.0);
        pidfController.setOutputLimits(-1.0, 1.0); // Motor power limits
        pidfController.setMaxIntegralSum(100.0); // Prevent integral windup

        // Example target position (e.g., encoder ticks)
        double targetPosition = 1000.0;

        waitForStart();

        while (opModeIsActive()) {
            // Get current encoder position
            double currentPosition = motor.getCurrentPosition();

            // Calculate motor power using PIDF
            double power = pidfController.calculate(targetPosition, currentPosition);

            // Apply power to motor
            motor.setPower(power);

            // Telemetry for debugging
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}