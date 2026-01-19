package org.firstinspires.ftc.teamcode.hardwareControl.actuators.Turret;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Turret.TurretConstants;

@Configurable
public class TurretController {

    private ServoImplEx turretServo;

    private TurretConstants turretConstants;

    private double TARGET_POSITION;
    private double lastTargetPosition = 0.0;
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


    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) return;

        setupConstants();
        this.telemetry = telemetry;
        initializeServo(hardwareMap);
        initialized = true;
    }

    private void setupConstants() {
        turretConstants = new TurretConstants();
    }

    private void initializeServo(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(ServoImplEx.class, turretConstants.name);
        MAX_DEGREES = turretConstants.maxDegrees;
        MIN_DEGREES = turretConstants.minDegrees;
        turretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void setTargetPosition(double targetPosition) {
        if ((targetPosition == lastTargetPosition) || (targetPosition - lastTargetPosition <= 1)) {
            return;
        }
        lastTargetPosition = targetPosition;

        targetPosition = targetPosition % 360;

        if(targetPosition < 0) {
            targetPosition += 360;
        }

        this.TARGET_POSITION = Range.clip(targetPosition, MIN_DEGREES, MAX_DEGREES);

        double servoPosition = this.TARGET_POSITION / 360;

        turretServo.setPosition(servoPosition);
        telemetry.addData("Turret Target", this.TARGET_POSITION);
    }

    public double getPosition() {
        return TARGET_POSITION;
    }

    public void update() {}

    public void reset() {
        if (!initialized) return;
        initialized = false;
//        turretServo.setPosition(0.5);
    }
}
