package org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.intake.IntakeTBHControllerConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.MotorConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.VelocityMotorConstantsBase;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.VelocityTBHControllerConstantBase;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.TBHController;


@Configurable
public class IntakeController {

    private DcMotorEx intakeMotor;

    private double START_VELOCITY;
    private double TOLERABLE_ERROR;
    private double targetVelocity;
    private final double REJECT_VELOCITY = -1;
    private final double RETAIN_VELOCITY = 0.3;



    /** TBH gain */
    public static double Kp = 0.0002;

    private double Kf_a;
    private double Kf_b;
    private double Kf_c;

    private TBHController tbhController;
    private boolean initialized = false;

    private static final IntakeController INSTANCE = new IntakeController();
    private Telemetry telemetry;

    private IntakeController() {}

    public static IntakeController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants() {
        try {
            Class.forName(IntakeConstants.class.getName());
            Class.forName(IntakeTBHControllerConstants.class.getName());
        } catch (ClassNotFoundException ignored) {}
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) return;

        setupConstants();
        this.telemetry = telemetry;

        initializeMotor(hardwareMap);
        initializeConstants();
        initializeTBH();

        initialized = true;
    }

    private void initializeMotor(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, VelocityMotorConstantsBase.frontMotorName);

        MotorConfigurationType type = intakeMotor.getMotorType().clone();
        type.setTicksPerRev(MotorConstants.ticksPerRev);
        type.setGearing(MotorConstants.gearing);
        type.setAchieveableMaxRPMFraction(MotorConstants.achievableMaxRPMFraction);

        intakeMotor.setMotorType(type);
        intakeMotor.setMode(MotorConstants.resetMode);
        intakeMotor.setMode(MotorConstants.mode);
        intakeMotor.setDirection(MotorConstants.direction);
    }

    private void initializeConstants() {
        START_VELOCITY = MotorConstants.startPosition;
        TOLERABLE_ERROR = MotorConstants.tolerableError;

        Kf_a = VelocityTBHControllerConstantBase.FRONT_Kf_a;
        Kf_b = VelocityTBHControllerConstantBase.FRONT_Kf_b;
        Kf_c = VelocityTBHControllerConstantBase.FRONT_Kf_c;
    }

    private void initializeTBH() {
        tbhController = new TBHController(Kp, Kf_a, Kf_b, Kf_c, telemetry);
    }

    /** RPM */
    public double getCurrentVelocity() {
        return intakeMotor.getVelocity(AngleUnit.DEGREES) / 6.0;
    }

    public void spinToTargetVelocity(double newTargetVelocity) {
        if (newTargetVelocity != targetVelocity) {
            tbhController.reset();
            targetVelocity = newTargetVelocity;
        }

        double currentVelocity = getCurrentVelocity();

        double power = tbhController.calculate(targetVelocity, currentVelocity);
        intakeMotor.setPower(power);

        telemetry.addData("Intake Target RPM", targetVelocity);
        telemetry.addData("Intake Current RPM", currentVelocity);
        telemetry.addData("Intake Power", power);
    }


    public void retainArtifacts() {
        spinToTargetVelocity(RETAIN_VELOCITY);
    }

    public void rejectArtifacts() {
        spinToTargetVelocity(REJECT_VELOCITY);
    }

    public boolean isOnTarget() {
        return Math.abs(targetVelocity - getCurrentVelocity()) <= TOLERABLE_ERROR;
    }

    public void reset() {
        if (!initialized) return;

        tbhController.reset();
        intakeMotor.setMode(MotorConstants.resetMode);
        intakeMotor.setMode(MotorConstants.mode);
        intakeMotor.setDirection(MotorConstants.direction);

        initialized = false;
    }

    public void update() {
        double currentVelocity = getCurrentVelocity();

        double power = tbhController.calculate(targetVelocity, currentVelocity);
        intakeMotor.setPower(power);

        telemetry.addData("Intake Target RPM", targetVelocity);
        telemetry.addData("Intake Current RPM", currentVelocity);
        telemetry.addData("Intake Power", power);
    }
}

