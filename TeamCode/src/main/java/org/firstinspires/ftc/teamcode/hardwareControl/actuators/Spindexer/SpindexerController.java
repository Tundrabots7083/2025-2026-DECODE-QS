package org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer.SpindexerConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer.SpindexerPIDFControllerConstants;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.MotionProfiler;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.PIDFController;

public class SpindexerController {

    private static final SpindexerController INSTANCE = new SpindexerController();
    private static final long RECOVERY_TIMEOUT_MS = 200; // ms per stage
    private static final double RECOVERY_OFFSET_DEG = 15; // backoff amount
    // Latched stuck flag; controller owns this entirely
    public boolean isStuck = false;
    private DcMotorEx spindexerMotor;
    private SpindexerConstants spindexerConstants;
    private SpindexerPIDFControllerConstants pidfControllerConstants;
    private double lastTargetPosition;
    private double TOLERABLE_ERROR;
    private double TOLERABLE_VELOCITY_ERROR;
    private double LAST_POWER;
    private double DEGREE_OFFSET;
    private double MAX_INTEGRAL_SUM;
    private double GEARING;
    private double MAX_VELOCITY;
    private double MAX_ACCELERATION;
    private double startTime;
    private double currentTime;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private PIDFController pidfController;
    private MotionProfiler motionProfiler;
    private boolean initialized = false;
    private boolean isSpinningSlowly = false;
    private Telemetry telemetry;
    private SpindexerState spindexerState = SpindexerState.IDLE;
    private double recoveryTarget;
    private double savedGoalPosition;


    private SpindexerController() {
    }

    public static SpindexerController getInstance() {
        return INSTANCE;
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) return;

        setupConstants();
        this.telemetry = telemetry;

        initializeMotor(hardwareMap);
        initializeLocalVariablesWithConstants();
        initializePIDFController();

        initialized = true;
    }

    private void setupConstants() {
        spindexerConstants = new SpindexerConstants();
        pidfControllerConstants = new SpindexerPIDFControllerConstants();
    }

    private void initializeMotor(HardwareMap hardwareMap) {
        spindexerMotor = hardwareMap.get(DcMotorEx.class, spindexerConstants.name);

        MotorConfigurationType motorConfigurationType =
                spindexerMotor.getMotorType().clone();

        motorConfigurationType.setTicksPerRev(spindexerConstants.ticksPerRev);
        motorConfigurationType.setAchieveableMaxRPMFraction(spindexerConstants.achievableMaxRPMFraction);
        this.GEARING = spindexerConstants.gearing;

        spindexerMotor.setMotorType(motorConfigurationType);
        spindexerMotor.setMode(spindexerConstants.resetMode);
        spindexerMotor.setMode(spindexerConstants.runMode);
        spindexerMotor.setDirection(spindexerConstants.direction);
    }

    private void initializeLocalVariablesWithConstants() {
        TOLERABLE_ERROR = spindexerConstants.tolerableError;
        TOLERABLE_VELOCITY_ERROR = spindexerConstants.tolerableVelocityError;
        DEGREE_OFFSET = spindexerConstants.degreeOffset;
        MAX_INTEGRAL_SUM = pidfControllerConstants.maxIntegralSum;
        MAX_VELOCITY = pidfControllerConstants.maxVelocity;
        MAX_ACCELERATION = pidfControllerConstants.maxAcceleration;

        kP = pidfControllerConstants.kp;
        kI = pidfControllerConstants.ki;
        kD = pidfControllerConstants.kd;
        kF = pidfControllerConstants.kf;
    }

    private void initializePIDFController() {
        pidfController = new PIDFController(kP, kI, kD, kF);
        pidfController.setMaxIntegralSum(MAX_INTEGRAL_SUM);
        motionProfiler = new MotionProfiler(MAX_VELOCITY, MAX_ACCELERATION);
    }

    public void hardwareReset() {
        spindexerMotor.setMode(spindexerConstants.resetMode);
        spindexerMotor.setMode(spindexerConstants.runMode);
    }


    public void moveToPosition(double targetPosition) {

        // Only generate new motion profile if not in recovery
        if (spindexerState == SpindexerState.IDLE || spindexerState == SpindexerState.MOVING_TO_TARGET && lastTargetPosition != targetPosition) {
            pidfController.reset();
            double startPosition = getPosition();
            motionProfiler.generateProfile(startPosition, targetPosition);
            startTime = System.currentTimeMillis();
            spindexerState = SpindexerState.MOVING_TO_TARGET;

            // Always store goal
            lastTargetPosition = targetPosition;
        }


        update();
    }

    public boolean isOnTarget() {
        boolean isInDeadband = Math.abs(lastTargetPosition - getPosition()) <= TOLERABLE_ERROR;
        boolean isVelocityLow = Math.abs(spindexerMotor.getVelocity(AngleUnit.DEGREES)) <= TOLERABLE_VELOCITY_ERROR;
        boolean isRecovering = spindexerState == SpindexerState.RECOVERY_BACKOFF || spindexerState == SpindexerState.RECOVERY_RETURN;
        return isInDeadband && isVelocityLow && !isRecovering;
    }

    public void spinSlowly() {
        spindexerMotor.setPower(0.3);
        isSpinningSlowly = true;
    }

    public void stop() {
        spindexerMotor.setPower(0);
        pidfController.reset();
        isSpinningSlowly = false;
    }

    public double getPosition() {
        double currentAngle = (spindexerMotor.getCurrentPosition() / spindexerConstants.ticksPerRev) * 360 * GEARING + DEGREE_OFFSET;
        return currentAngle;
    }

    public double getRPMVelocity() {
        double currentDegPerSecond = spindexerMotor.getVelocity(AngleUnit.DEGREES);
        return currentDegPerSecond / 6; // Convert to RPM
    }

    public double getTargetPosition() {
        return lastTargetPosition;
    }

    public int getSlotPosition() {
        double reducedAngle = getPosition() % 360;
        if (reducedAngle < 0) {
            reducedAngle += 360;
        }

        int slot = (int) Math.round(reducedAngle / 120.0) % 3;
        return slot;
    }

    public boolean isBusy() {
        return spindexerMotor.isBusy();
    }

    public void update() {
        if (isSpinningSlowly) {
            return;
        }

        currentTime = System.currentTimeMillis();
        double currentPosition = getPosition();
        double elapsedTime = (currentTime - startTime) / 1000.0;
//        telemetry.addData("[SPINDEXER] CurrentPos", currentPosition);
//        telemetry.addData("[SPINDEXER] TargetPose", getTargetPosition());

        // ----- Normal motion -----
        MotionProfiler.MotionState motionState = motionProfiler.getMotionState(elapsedTime);

        double power = pidfController.calculate(motionState.position, currentPosition);
        double feedForwardPower = power + kF * motionState.velocity;
        feedForwardPower = Range.clip(feedForwardPower, -0.5, 1);

        if (Math.abs(LAST_POWER - feedForwardPower) > 0.08) {
            spindexerMotor.setPower(feedForwardPower);
//            telemetry.addData("[SPINDEXER] Power", feedForwardPower);
            LAST_POWER = feedForwardPower;
        }

        // ----- Stuck detection and recovery -----
        detectStuck(elapsedTime);

        switch (spindexerState) {
            case RECOVERY_BACKOFF:
                // Step 1: move to backoff target
                if (isOnTarget() || System.currentTimeMillis() - startTime > RECOVERY_TIMEOUT_MS) {
                    spindexerState = SpindexerState.RECOVERY_RETURN;
                    startTime = System.currentTimeMillis();
                    pidfController.reset();
                    motionProfiler.generateProfile(getPosition(), savedGoalPosition);
                }
                break;

            case RECOVERY_RETURN:
                // Step 2: move back to original goal
                if (isOnTarget() || System.currentTimeMillis() - startTime > RECOVERY_TIMEOUT_MS) {
                    spindexerState = SpindexerState.MOVING_TO_TARGET;
                    isStuck = false; // auto-clear flag
                }
                break;

            case MOVING_TO_TARGET:
            case IDLE:
            default:
                break;
        }

    }

    private void detectStuck(double elapsedTime) {
        double currentVelocity = getRPMVelocity();
        double predictedVelocity = (LAST_POWER - (Math.signum(LAST_POWER) * 0.07)) * 240;
        double predictedVelocityError = predictedVelocity - currentVelocity;

        if (!isStuck &&
                predictedVelocityError >= 800 &&
                Math.abs(currentVelocity) < 20 &&
                Math.abs(LAST_POWER) > 0.1 &&
                elapsedTime > 0.8) {

            // Latch stuck flag
            isStuck = true;

            // Initialize recovery FSM
            startTime = System.currentTimeMillis();
            savedGoalPosition = lastTargetPosition;
            recoveryTarget = getPosition() - RECOVERY_OFFSET_DEG;
            pidfController.reset();
            motionProfiler.generateProfile(getPosition(), recoveryTarget);

            spindexerState = SpindexerState.RECOVERY_BACKOFF;

            // Stop motor to prevent stress
            spindexerMotor.setPower(0.0);
        }
    }

    public void setDegreeOffset(double newOffset) {
        this.DEGREE_OFFSET = newOffset;
    }

    public void testSpindexer(double power) {
        spindexerMotor.setPower(power);
    }

    public void setKf(double newkF) {
        pidfController.setPIDF(kP, kI, kD, newkF);
    }

    public void reset() {
        if (!initialized) return;

        spindexerMotor.setMode(spindexerConstants.resetMode);
        spindexerMotor.setMode(spindexerConstants.runMode);
        spindexerMotor.setDirection(spindexerConstants.direction);

        lastTargetPosition = getPosition();
        pidfController.reset();
        isStuck = false;
        initialized = false;
        spindexerState = SpindexerState.IDLE;
    }

    // ----- Recovery FSM -----
    private enum SpindexerState {
        IDLE,
        MOVING_TO_TARGET,
        RECOVERY_BACKOFF,
        RECOVERY_RETURN
    }
}
