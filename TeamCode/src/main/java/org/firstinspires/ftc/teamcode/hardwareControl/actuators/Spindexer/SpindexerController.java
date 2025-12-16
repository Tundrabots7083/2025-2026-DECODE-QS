package org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer.SpindexerConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Spindexer.SpindexerPIDFControllerConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.MotorConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.PIDFControllerConstants;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.PIDFController;

    @Configurable
    public class SpindexerController {

        private DcMotorEx spindexerMotor;

        private double TARGET_POSITION;
        private double TOLERABLE_ERROR;

        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0;

        private PIDFController pidfController;

        private boolean initialized = false;

        // Singleton instance
        private static final SpindexerController INSTANCE = new SpindexerController();
        private Telemetry telemetry;

        private SpindexerController() {}

        public static SpindexerController getInstance() {
            return INSTANCE;
        }

        private static void setupConstants() {
            try {
                Class.forName(SpindexerConstants.class.getName());
                Class.forName(SpindexerPIDFControllerConstants.class.getName());
            } catch (ClassNotFoundException e) {
                // ignored intentionally
            }
        }

        public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
            if (initialized) return;

            setupConstants();
            this.telemetry = telemetry;

            initializeMotor(hardwareMap);
            initializeLocalVariablesWithConstants();
            initializePIDFController();

            initialized = true;
        }

        private void initializeMotor(HardwareMap hardwareMap) {
            spindexerMotor = hardwareMap.get(DcMotorEx.class, MotorConstants.name);

            MotorConfigurationType motorConfigurationType =
                    spindexerMotor.getMotorType().clone();

            motorConfigurationType.setTicksPerRev(MotorConstants.ticksPerRev);
            motorConfigurationType.setGearing(MotorConstants.gearing);
            motorConfigurationType.setAchieveableMaxRPMFraction(
                    MotorConstants.achievableMaxRPMFraction
            );

            spindexerMotor.setMotorType(motorConfigurationType);
            spindexerMotor.setMode(MotorConstants.resetMode);
            spindexerMotor.setMode(MotorConstants.mode);
            spindexerMotor.setDirection(MotorConstants.direction);
        }

        private void initializeLocalVariablesWithConstants() {
            TOLERABLE_ERROR = MotorConstants.tolerableError;

            kP = PIDFControllerConstants.kp;
            kI = PIDFControllerConstants.ki;
            kD = PIDFControllerConstants.kd;
            kF = PIDFControllerConstants.kf;
        }

        private void initializePIDFController() {
            pidfController = new PIDFController(
                    kP,
                    kI,
                    kD,
                    kF            );
        }


        public void moveToPosition(double targetPositionTicks) {

            if (targetPositionTicks != TARGET_POSITION) {
                pidfController.reset();
            }

            TARGET_POSITION = targetPositionTicks;

            double currentPosition = spindexerMotor.getCurrentPosition();

            // PID + Feedforward handled internally
            double power = pidfController.calculate(TARGET_POSITION, currentPosition);

            spindexerMotor.setPower(power);

            telemetry.addData("Spindexer Target", TARGET_POSITION);
            telemetry.addData("Spindexer Position", currentPosition);
            telemetry.addData("Spindexer Power", power);
        }

        public boolean isOnTarget() {
            return Math.abs(TARGET_POSITION - spindexerMotor.getCurrentPosition())
                    <= TOLERABLE_ERROR;
        }

        public void stop() {
            spindexerMotor.setPower(0);
            pidfController.reset();
        }


        public double getCurrentPosition() {
            return spindexerMotor.getCurrentPosition();
        }

        public boolean isBusy() {
            return spindexerMotor.isBusy();
        }

        public void update() {
        }

        public void reset() {
            if (!initialized) return;

            spindexerMotor.setMode(MotorConstants.resetMode);
            spindexerMotor.setMode(MotorConstants.mode);
            spindexerMotor.setDirection(MotorConstants.direction);

            pidfController.reset();
            initialized = false;
        }
    }
