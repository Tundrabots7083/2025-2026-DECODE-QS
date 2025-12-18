package org.firstinspires.ftc.teamcode.hardwareControl.actuators.Turret;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ServoConstants;

@Configurable
public class TurretController {

    private ServoImplEx turretServo;

    private double TARGET_POSITION;
    private double MAX_DEGREES;
    private double MIN_DEGREES;

    private boolean initialized = false;

    // Singleton instance
    private static final TurretController INSTANCE = new TurretController();
    private Telemetry telemetry;

    private TurretController() {}

    public static TurretController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants() {
        try {
            Class.forName(TurretController.class.getName());
        } catch (ClassNotFoundException e) {
            // ignored intentionally
        }
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        if (initialized) return;

        setupConstants();
        this.telemetry = telemetry;

        initializeServo(hardwareMap);

        initialized = true;
    }

    private void initializeServo(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(ServoImplEx.class, ServoConstants.name);
        MAX_DEGREES = ServoConstants.maxDegrees;
        MIN_DEGREES = ServoConstants.minDegrees;
    }

    public void setTargetPosition(double targetPosition) {
        targetPosition = targetPosition % 360;

        if(targetPosition < 0) {
            targetPosition += 360;
        }

        TARGET_POSITION = Range.clip(targetPosition, MIN_DEGREES, MAX_DEGREES);

        double servoPosition = TARGET_POSITION / 360;

        turretServo.setPosition(servoPosition);
        telemetry.addData("Turret Target", TARGET_POSITION);
    }

    public double getPosition() {
        return TARGET_POSITION;
    }


    public void update() {}

    public void reset() {
        if (!initialized) return;
        turretServo.setPosition(0.5);
    }
}
