package org.firstinspires.ftc.teamcode.opModes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.MotionProfiler;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.PIDFController;

@TeleOp(name="Motion Profiled PIDF OpMode")
public class MotionProfiledOpMode extends LinearOpMode {
    private DcMotor motor;
    private PIDFController pidfController;
    private MotionProfiler motionProfiler;
    private double startTime;

    @Override
    public void runOpMode() {
        // Initialize motor
        motor = hardwareMap.get(DcMotor.class, "testMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize PIDF controller (tune these gains!)
        pidfController = new PIDFController(0.01, 0.0001, 0.1, 0.0);
        pidfController.setOutputLimits(-1.0, 1.0);
        pidfController.setMaxIntegralSum(100.0);

        // Initialize motion profiler (tune these constraints!)
        motionProfiler = new MotionProfiler(1000.0, 2000.0); // e.g., ticks/s, ticks/s^2
        double initialPosition = 0.0;
        double targetPosition = 2000.0; // e.g., encoder ticks

        // Generate motion profile
        motionProfiler.generateProfile(initialPosition, targetPosition);

        waitForStart();
        startTime = System.nanoTime() / 1_000_000_000.0; // Start time in seconds

        while (opModeIsActive()) {
            // Get current time relative to start
            double currentTime = System.nanoTime() / 1_000_000_000.0 - startTime;

            // Get motion profile setpoint
            MotionProfiler.MotionState state = motionProfiler.getMotionState(currentTime);
            double setpointPosition = state.position;

            // Get current encoder position
            double currentPosition = motor.getCurrentPosition();

            // Calculate motor power using PIDF
            double power = pidfController.calculate(setpointPosition, currentPosition);

            // Apply power to motor
            motor.setPower(power);

            // Telemetry for debugging
            telemetry.addData("Target Position", setpointPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Power", power);
            telemetry.addData("Profile Time", currentTime);
            telemetry.update();

            // Stop when motion is complete
            if (currentTime > motionProfiler.getTotalTime()) {
                motor.setPower(0);
                break;
            }
        }
    }
}