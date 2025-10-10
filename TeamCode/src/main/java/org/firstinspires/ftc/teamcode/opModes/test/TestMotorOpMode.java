package org.firstinspires.ftc.teamcode.opModes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.hardwareControl.sensors.intakeDistanceSensor.IntakeDistanceSensorController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.frontLeftCamera.FrontLeftCameraController;

@Autonomous(name="TestMotor/servo", group="specimens")
public class TestMotorOpMode extends LinearOpMode
{

    private DcMotorEx testMotor;

    private Servo servo;

    public static double motorTargetPosition =1000;
    public static double motorPower =0.1;
    public static DcMotorEx.Direction motorDirection= DcMotorEx.Direction.FORWARD;
    ShooterController shooterController;
    IntakeDistanceSensorController frontDistanceSensorController;
    FrontLeftCameraController frontLeftCameraController;

    double startAngle;
    double targetAngle = 90.00;



    public static double servoPosition =0;
    public static Servo.Direction servoDirection= Servo.Direction.FORWARD;



    private DistanceSensor distanceSensor;

    private long count=0;


    @Override
    public void runOpMode()
    {

        telemetry.clearAll();
        telemetry.addData("TestMotorOpMode", "runOpMode started");
        telemetry.update();
        waitForStart();
        /// Shoulder
//        this.shoulderController = ShoulderController.getInstance();
//
//        this.shoulderController.reset();
//        this.shoulderController.initialize(hardwareMap, telemetry, this);
        /// End Shoulder

        /// Front Distance Sensor
        frontDistanceSensorController = IntakeDistanceSensorController.getInstance();
        frontDistanceSensorController.initialize(hardwareMap, telemetry, this);

        /// End Front Distance Sensor

        /// Front Left Camera
        frontLeftCameraController =FrontLeftCameraController.getInstance();
        frontLeftCameraController.initialize(hardwareMap, telemetry, this);

        /// End Front Left Camera
/*
        testMotor =  hardwareMap.get(DcMotorEx.class, "testMotor");
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor.setDirection(motorDirection);


        servo = hardwareMap.get(Servo.class, "testServo");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "testDistance");
*/
        while (opModeIsActive())
        {
            count++;
            telemetry.addData("TestMotorOpMode", "runOpMode while started count: %d",count);
            telemetry.update();

            // Get current encoder position
           // double currentPosition = testMotor.getCurrentPosition();

            // Calculate motor power using PIDF
           // double power = pidfController.calculate(targetPosition, currentPosition);

            // Apply power to motor
            //motor.setPower(power);
           // testMotor.setPower(motorPower);

//            double currentPosition =shoulderController.getCurrentPosition();
//            shoulderController.spinToTargetVelocity(this.targetAngle);
//
//            Telemetry for debugging
//            telemetry.addData("Target Angle", targetAngle);
//            telemetry.addData("Current Angle", currentPosition);
//            telemetry.addData("Power", motorPower);
//            telemetry.update();
//
            /// Front Distance Sensor

            frontDistanceSensorController.getCurrentDistance();
            /// End Front Distance Sensor

            /// Front left camera
            frontLeftCameraController.getCurrentDetections();

            /// End Front Left Camera




           // servo.setDirection(Servo.Direction.FORWARD);
           // servo.setPosition(servoPosition);

           // telemetry.addData("Distance (CM)", this.getDistance(DistanceUnit.CM));
           // telemetry.addData("Distance (IN)", this.getDistance(DistanceUnit.INCH));
           // telemetry.update();

        }
    }

    public double getDistance(DistanceUnit du){
        return this.distanceSensor.getDistance(du);
    }

}
