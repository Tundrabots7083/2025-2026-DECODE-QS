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
            motorConfigurationType.setGearing(spindexerConstants.gearing);
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
                motionProfiler.generateProfile(spindexerMotor.getCurrentPosition(), targetPosition);
                startTime = System.currentTimeMillis();
            }

            currentTime = System.currentTimeMillis();
            lastTargetPosition = targetPosition;

            double currentPosition = getPosition();
            double elapsedTime = currentTime - startTime;

            double power = pidfController.calculate(motionProfiler.getMotionState(elapsedTime).position, currentPosition);

            spindexerMotor.setPower(power);

            LAST_POWER = power;

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

            double currentPosition = getPosition();
            double elapsedTime = currentTime - startTime;

            double power = pidfController.calculate(motionProfiler.getMotionState(elapsedTime).position, currentPosition);

            spindexerMotor.setPower(power);

            telemetry.addData("Spindexer Target", lastTargetPosition);
            telemetry.addData("Spindexer Position", currentPosition);
            telemetry.addData("Spindexer Power", power);

            if(Math.abs(LAST_POWER - power) > 0.001) {
                spindexerMotor.setPower(power);
                LAST_POWER = power;
            }

        }

        public void setDegreeOffset(double newOffset) {
            this.DEGREE_OFFSET = newOffset;
        }

        public void setKf(double newkF) {
            pidfController.setPIDF(kP, kI, kD, newkF);
        }

        public void reset() {
            if (!initialized) return;

            spindexerMotor.setMode(spindexerConstants.resetMode);
            spindexerMotor.setMode(spindexerConstants.runMode);
            spindexerMotor.setDirection(spindexerConstants.direction);

            pidfController.reset();
            initialized = false;
        }
    }
