package org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.SimpleVelocityController;


public class IntakeController {

    private DcMotorEx intakeMotor;

    private IntakeConstants intakeConstants;

    private double TOLERABLE_ERROR;
    private double targetVelocity;
    private double lastPower = 0.0;


    /**
     * velocity gain
     */
    private double Kp = 0.00025;

    private SimpleVelocityController velocityController;
    private boolean initialized = false;

    private static final IntakeController INSTANCE = new IntakeController();
    private Telemetry telemetry;

    private IntakeController() {}

    public static IntakeController getInstance() {
        return INSTANCE;
    }


    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) return;

        setupConstants();
        this.telemetry = telemetry;

        initializeMotor(hardwareMap);
        initializeConstants();
        initializeVelControl();

        initialized = true;
    }

    private void setupConstants() {
        intakeConstants = new IntakeConstants();
    }
    
    private void initializeMotor(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, intakeConstants.motorName);

        MotorConfigurationType type = intakeMotor.getMotorType().clone();
        type.setTicksPerRev(intakeConstants.ticksPerRev);
        type.setGearing(intakeConstants.gearing);
        type.setAchieveableMaxRPMFraction(intakeConstants.achievableMaxRPMFraction);

        intakeMotor.setMotorType(type);
        intakeMotor.setMode(intakeConstants.resetMode);
        intakeMotor.setMode(intakeConstants.mode);
        intakeMotor.setDirection(intakeConstants.motorDirection);
    }

    private void initializeConstants() {
        TOLERABLE_ERROR = intakeConstants.tolerableError;
    }

    private void initializeVelControl() {
        velocityController = new SimpleVelocityController(Kp, telemetry);
    }

    /** RPM */
    public double getCurrentVelocity() {
        return intakeMotor.getVelocity(AngleUnit.DEGREES) / 6.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void spinToTargetVelocity(double newTargetVelocity) {
        if (newTargetVelocity != targetVelocity) {
            velocityController.reset();
            targetVelocity = newTargetVelocity;
        }

        update();
    }

    public boolean isOnTarget() {
        return Math.abs(targetVelocity - getCurrentVelocity()) <= TOLERABLE_ERROR;
    }

    public void reset() {
        if (!initialized) return;

        velocityController.reset();
        intakeMotor.setMode(intakeConstants.resetMode);
        intakeMotor.setMode(intakeConstants.mode);
        intakeMotor.setDirection(intakeConstants.motorDirection);

        initialized = false;
    }

    public void update() {
        double currentVelocity = getCurrentVelocity();

        double power = velocityController.calculate(targetVelocity, currentVelocity);

        if (Math.abs(lastPower - power) > 0.004) {
            intakeMotor.setPower(power);
            lastPower = power;
        }


//        telemetry.addData("Intake Target RPM", targetVelocity);
//        telemetry.addData("Intake Current RPM", currentVelocity);
//        telemetry.addData("Intake Power", power);
    }
}

