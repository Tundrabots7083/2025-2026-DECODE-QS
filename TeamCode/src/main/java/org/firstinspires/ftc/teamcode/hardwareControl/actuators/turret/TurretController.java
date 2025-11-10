package org.firstinspires.ftc.teamcode.hardwareControl.actuators.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.turret.TurretMotionProfilerConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.turret.TurretPIDFControllerConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.MotorConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.PIDFControllerConstants;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.MotionProfiler;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.PIDFController;

public class TurretController {
    private DcMotorEx turretMotor;

    private double MAX_POSITION;
    private double MIN_POSITION;
    private double START_POSITION;
    private double targetPosition;
    private double lastPosition;
    private double TOLERABLE_ERROR;
    private double FEED_FORWARD;

    private double INCHES_PER_REV;

    public static double Kp = 0.01;
    public static double Ki = 0.001;
    public static double Kd = 0.0;
    public static double Kf = 0.0;


    /// motion control
    private PIDFController pidfController;

    private MotionProfiler motionProfiler;

    private double currentPosition=0.0;
    public static double mpMaxVelocity = 500;  /// deg/s
    public static double mpMaxAcceleration = 200;   /// deg/s**2
    private double startTime;
    ///

    private boolean initialized = false;
    private boolean isMotionProfileGenerated = false;

    // Private static instance (eager initialization)
    private static final TurretController INSTANCE = new TurretController();
    private LinearOpMode opMode;
    private Telemetry telemetry;

    // Private constructor to prevent instantiation
    private TurretController() {
        // Initialize hardware, state, or configuration here

    }
    // Public method to access the singleton instance
    public static TurretController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants(){
        try {
            Class.forName(TurretConstants.class.getName());
            Class.forName(TurretPIDFControllerConstants.class.getName());
            Class.forName(TurretMotionProfilerConstants.class.getName());
        } catch (ClassNotFoundException e) {
            //e.printStackTrace();
        }
    }
    // Initialization method — must be called once at the beginning
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        if (initialized) {
            return;
        }
        setupConstants();
        this.opMode = opMode;
        this.telemetry  = telemetry;

        initializeMotor(hardwareMap);
        initializeLocalVariablesWithConstants();
        initializePDFController();
        initializeMotionProfiler();

        initialized = true;
    }

    private void initializeMotor(HardwareMap hardwareMap){
        turretMotor =  hardwareMap.get(DcMotorEx.class, MotorConstants.name);
        MotorConfigurationType motorConfigurationType = turretMotor.getMotorType().clone();
        motorConfigurationType.setTicksPerRev(MotorConstants.ticksPerRev);
        motorConfigurationType.setGearing(MotorConstants.gearing);
        motorConfigurationType.setAchieveableMaxRPMFraction(MotorConstants.achievableMaxRPMFraction);
        turretMotor.setMotorType(motorConfigurationType);
        turretMotor.setMode(MotorConstants.resetMode);
        turretMotor.setMode(MotorConstants.mode);
        turretMotor.setDirection(MotorConstants.direction);
    }

    private void initializeLocalVariablesWithConstants(){
        START_POSITION = MotorConstants.startPosition;
        lastPosition = START_POSITION;
        MAX_POSITION = MotorConstants.maxPosition;
        MIN_POSITION = MotorConstants.minPosition;
        TOLERABLE_ERROR = MotorConstants.tolerableError;
        FEED_FORWARD = MotorConstants.feedforward;
        INCHES_PER_REV = MotorConstants.inchesPerRev;
    }

    private void initializePDFController(){
  /*        Kp= PIDFControllerConstants.kp;
        Ki=PIDFControllerConstants.ki;
        Kd=PIDFControllerConstants.kd;*/

        pidfController = new PIDFController(Kp, Ki, Kd, Kf);
        pidfController.setOutputLimits(PIDFControllerConstants.motorMinPowerLimit, PIDFControllerConstants.motorMaxPowerLimit); // Motor power limits
        pidfController.setMaxIntegralSum(PIDFControllerConstants.maxIntegralSum); // Prevent integral windup
    }

    private void initializeMotionProfiler(){
        // Initialize motion profiler (tune these constraints!)
        motionProfiler = new MotionProfiler(mpMaxVelocity, mpMaxAcceleration); // e.g., ticks/s, ticks/s^2

    }

    private void generateMotionProfile(double startPosition, double targetPosition){
        if(isMotionProfileGenerated){
            return;
        }

        // Generate motion profile
        motionProfiler.generateProfile(startPosition, targetPosition);

        startTime = System.nanoTime() / 1_000_000_000.0; // Start time in seconds

        isMotionProfileGenerated=true;

        telemetry.addData("TurretCtrl.generateMotionProfile startPosition", startPosition);
        telemetry.addData("TurretCtrl.generateMotionProfile targetPosition", targetPosition);
        telemetry.update();
    }
    public void reset() {
        if(initialized) {
            if(this.isBusy()) {
                this.moveToTargetPosition(START_POSITION);
            } else {
                turretMotor.setMode(MotorConstants.resetMode);
                turretMotor.setMode(MotorConstants.mode);
                turretMotor.setDirection(MotorConstants.direction);
                isMotionProfileGenerated=false;
            }
            initialized = false;
        }
    }



    // Example method
    public double getCurrentPosition() {
        MotorConfigurationType motorType = turretMotor.getMotorType();
        double ticksPerRev = motorType.getTicksPerRev();
        double currentTicks = turretMotor.getCurrentPosition();
        double rotations = currentTicks / ticksPerRev;
        double currentPosition = rotations * INCHES_PER_REV + START_POSITION;


        return rotations * INCHES_PER_REV + START_POSITION;
    }

    public void moveToTargetPosition(double newTargetPosition){
        this.targetPosition = newTargetPosition;

        generateMotionProfile(lastPosition, targetPosition);


        // Get current time relative to start
        double currentTime = System.nanoTime() / 1_000_000_000.0 - startTime;

        // Get motion profile setpoint
        MotionProfiler.MotionState state = motionProfiler.getMotionState(currentTime);
        double setpointPosition = state.position;

        // Get current encoder position
        currentPosition = this.getCurrentPosition();

        // Calculate motor pidPower using PIDF
        double pidPower = pidfController.calculate(setpointPosition, currentPosition);

        // Apply pidPower to motor
        turretMotor.setPower(pidPower);

        telemetry.addData("TurretCtrl Target position", newTargetPosition);
        telemetry.addData("TurretCtrl Current position", currentPosition);
        telemetry.addData("TurretCtrl setpointPosition", setpointPosition);
        telemetry.addData("TurretCtrl pidPower", pidPower);
        telemetry.update();

    }

    public boolean isOnTarget(){
        double currentPosition = this.getCurrentPosition();
        boolean isOnTarget = ((Math.abs(targetPosition - currentPosition) <= TOLERABLE_ERROR));
        if (isOnTarget){
            lastPosition = currentPosition;
            isMotionProfileGenerated=false;
        }
        return isOnTarget;
    }

    public void update(){

    }

    public boolean isBusy(){
        return turretMotor.isBusy();
    }


}

/*usage Example*/
