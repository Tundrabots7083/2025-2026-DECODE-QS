package org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer.SpindexerConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer.SpindexerPIDFControllerConstants;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.MotionProfiler;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.PIDFController;

    public class SpindexerController {

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

        // Singleton instance
        private static final SpindexerController INSTANCE = new SpindexerController();
        private Telemetry telemetry;

        private SpindexerController() {}

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
            pidfController = new PIDFController(
                    kP,
                    kI,
                    kD,
                    kF            );

            pidfController.setMaxIntegralSum(MAX_INTEGRAL_SUM);
            motionProfiler = new MotionProfiler(MAX_VELOCITY, MAX_ACCELERATION);
        }

        public void hardwareReset() {
            spindexerMotor.setMode(spindexerConstants.resetMode);
            spindexerMotor.setMode(spindexerConstants.runMode);
        }


        public void moveToPosition(double targetPosition) {

            if (targetPosition != lastTargetPosition) {
                pidfController.reset();
                double startPosition = getPosition();
                motionProfiler.generateProfile(startPosition, targetPosition);
                startTime = System.currentTimeMillis();
                lastTargetPosition = targetPosition;
            }

                update();
        }

        public boolean isOnTarget() {
            boolean isInDeadband = Math.abs(lastTargetPosition - getPosition())
                    <= TOLERABLE_ERROR;
            boolean isVelocityLow = Math.abs(spindexerMotor.getVelocity(AngleUnit.DEGREES))
                    <= TOLERABLE_VELOCITY_ERROR;
            return isInDeadband && isVelocityLow;
        }

        public void spinSlowly() {
            spindexerMotor.setPower(0.3);
        }

        public void stop() {
            spindexerMotor.setPower(0);
            pidfController.reset();
        }
        public double getPosition() {
            double currentAngle = (spindexerMotor.getCurrentPosition() / spindexerConstants.ticksPerRev) * 360 * GEARING + DEGREE_OFFSET; //current position in degrees
            return currentAngle;
        }

        public double getRPMVelocity() {
            double currentDegPerSecond = spindexerMotor.getVelocity(AngleUnit.DEGREES);
            return currentDegPerSecond / 6; // Current RPM
        }

        public double getTargetPosition() {
            return lastTargetPosition;
        }

        public int getSlotPosition() {
            double currentAngle = getPosition() % 360;
            if (currentAngle < 0) {
                currentAngle += 360;
            }

            if (currentAngle < 120) {
                return 0;
            } else if (currentAngle < 240) {
                return 1;
            } else {
                return 2;
            }
        }

        public boolean isBusy() {
            return spindexerMotor.isBusy();
        }

        public void update() {

            currentTime = System.currentTimeMillis();
            double currentPosition = getPosition();
            double elapsedTime = (currentTime - startTime) / 1000.0; // Need seconds for Motion profile

            double error = getTargetPosition() - currentPosition;
            double power = pidfController.calculate(motionProfiler.getMotionState(elapsedTime).position, currentPosition);
            double feedForwardPower = power + kF * motionProfiler.getMotionState(elapsedTime).velocity;

            telemetry.addData("Spindexer Target", lastTargetPosition);
            telemetry.addData("Spindexer Position", currentPosition);
            telemetry.addData("Spindexer PID Power", power);
            telemetry.addData("Spindexer kF Power", feedForwardPower);
            telemetry.addData("Spindexer Error", error);
            telemetry.addData("Spindexer Velocity", getRPMVelocity());


            if (Math.abs(LAST_POWER - feedForwardPower) > 0.01) {
                spindexerMotor.setPower(feedForwardPower);
                LAST_POWER = feedForwardPower;
            }

            double currentVelocity = getRPMVelocity();
            double predictedVelocity = (feedForwardPower - (Math.signum(feedForwardPower) * 0.07)) * 240;
            double predictedVelocityError = predictedVelocity - currentVelocity;
            if (predictedVelocityError >= 100
                    && currentVelocity >= -0.01
                    && Math.abs(feedForwardPower) > 0.1
                    && elapsedTime > 0.5
                    && elapsedTime < motionProfiler.getTotalTime()) {
                spindexerMotor.setPower(0.0);
                System.out.println("SPINDEXER KF POWER THROW: " + predictedVelocity);
                System.out.println("SPINDEXER VELOCITY THROW: " + currentVelocity);
                System.out.println("SPINDEXER VELOCITY ERROR: " + predictedVelocityError);
                throw new RuntimeException("Spindexer is stuck!");
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
            initialized = false;
        }
    }
