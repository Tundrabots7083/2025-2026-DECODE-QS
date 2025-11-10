package org.firstinspires.ftc.teamcode.hardwareControl.actuators.elevator;

import static org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.MotorConstants.direction;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ServoConstants;

public class ElevatorController {
    private Servo elevatorServo;

    private double MAX_DEGREES;
    private double MIN_DEGREES;
    private long WAIT_TIME;
    private double START_POSITION;
    private double targetPosition;
    private double lastPosition;


    private boolean initialized = false;


    // Private static instance (eager initialization)
    private static final ElevatorController INSTANCE = new ElevatorController();
    private LinearOpMode opMode;
    private Telemetry telemetry;

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
    // Initialization method — must be called once at the beginning
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        if (initialized) {
            return;
            //throw new IllegalStateException("ArmController has already been initialized.");
        }
        setupConstants();
        this.opMode = opMode;
        this.telemetry  = telemetry;

        initializeServo(hardwareMap);
        initializeLocalVariablesWithConstants();

        initialized = true;
    }

    private void initializeServo(HardwareMap hardwareMap){
        elevatorServo =  hardwareMap.get(Servo.class, ServoConstants.name);
           }

    private void initializeLocalVariablesWithConstants(){
        START_POSITION = 0;
        lastPosition = START_POSITION;
        MAX_DEGREES =  ServoConstants.maxDegrees;
        MIN_DEGREES = ServoConstants.minDegrees;
        WAIT_TIME = ServoConstants.waitTime;
    }




    public void reset() {
        if(initialized) {
            this.moveToTargetPosition(START_POSITION, Servo.Direction.FORWARD);
            initialized = false;
        }
    }



    // Example method
    public double getCurrentPosition() {
        return elevatorServo.getPosition();
    }

    public void moveToTargetPosition(double newTargetPosition, Servo.Direction direction){
        this.targetPosition = newTargetPosition;

        elevatorServo.setDirection(direction);
        elevatorServo.setPosition(newTargetPosition);

        opMode.sleep(WAIT_TIME);
    }

    public void update(){

    }

}

/*usage Example*/

