package org.firstinspires.ftc.teamcode.hardwareControl.actuators.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ServoConstants;



public class ElevatorController  {
    private boolean initialized = false;


    //debugging
    private LinearOpMode opMode;
    private Telemetry telemetry;
    //singleton
    private static final ElevatorController INSTANCE = new ElevatorController();
    private Servo elevatorServo;
    private double maxDegrees = 0;
    private double minDegrees = 0;
    private long waitTime = 150;
    private double upPosition = 100;
    private double downPosition = 0;

    // Private constructor to prevent instantiation
    private ElevatorController() {
        // Initialize hardware, state, or configuration here

    }

    // Public method to access the singleton instance
    public static ElevatorController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants(){
        try {
            Class.forName(ElevatorConstants.class.getName());
        } catch (ClassNotFoundException e) {
            //e.printStackTrace();
        }
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        if (initialized) {
            return;
        }

        setupConstants();
        this.opMode = opMode;
        this.telemetry  = telemetry;


        initElevatorServo(hardwareMap);


        initialized = true;
    }

    public void reset() {

    }

    private void initElevatorServo(HardwareMap hardwareMap) {
        elevatorServo = hardwareMap.get(Servo.class, ServoConstants.name);

        maxDegrees = ServoConstants.maxDegrees;
        minDegrees = ServoConstants.minDegrees;
        waitTime = ServoConstants.waitTime;

    }
    public void moveUp() {
        elevatorServo.setPosition(upPosition);
        opMode.sleep(waitTime);
    }

    public void moveDown() {
        elevatorServo.setPosition(downPosition);
        opMode.sleep(waitTime);
    }

}