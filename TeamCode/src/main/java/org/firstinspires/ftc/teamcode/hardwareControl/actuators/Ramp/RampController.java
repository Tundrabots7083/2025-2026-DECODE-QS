package org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.Ramp.RampConstants;

@Configurable
public class RampController {

    private ServoImplEx rampServo;

    private RampConstants rampConstants;

    private double TARGET_POSITION;
    private double MAX_DEGREES;
    private double MIN_DEGREES;

    private boolean initialized = false;

    private boolean deployed = false;
    double startTime = -1;


    // Singleton instance
    private static final RampController INSTANCE = new RampController();
    private Telemetry telemetry;

    private RampController() {}

    public static RampController getInstance() {
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
        rampConstants = new RampConstants();
    }
    
    private void initializeServo(HardwareMap hardwareMap) {
        rampServo = hardwareMap.get(ServoImplEx.class, rampConstants.name);
        MAX_DEGREES = rampConstants.maxDegrees;
        MIN_DEGREES = rampConstants.minDegrees;
        rampServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void setTargetPosition(double targetPosition) {

        targetPosition = targetPosition % 360;

        if(targetPosition < 0) {
            targetPosition += 360;
        }

        TARGET_POSITION = Range.clip(targetPosition, MIN_DEGREES, MAX_DEGREES);

        double servoPosition = TARGET_POSITION / MAX_DEGREES;

        rampServo.setPosition(servoPosition);
//        telemetry.addData("Ramp Target", TARGET_POSITION);
//        telemetry.addData("Servo Target", servoPosition);

    }

    public double getPosition() {
        return TARGET_POSITION;
    }

    public void deploy() {
        if (!deployed && startTime < 0) {
            startTime = System.currentTimeMillis();
            setTargetPosition(120);
        }

        if (System.currentTimeMillis() - startTime > 400) {
            deployed = true;
            startTime = -1;
        }
    }

    public void store() {
        if (deployed && startTime < 0) {
            startTime = System.currentTimeMillis();
            setTargetPosition(0);
        }

        if (System.currentTimeMillis() - startTime > 400) {
            deployed = false;
            startTime = -1;
        }
    }

    public boolean isDeployed() {
        return deployed;
    }

    public void reset() {
        if (!initialized) return;
        initialized = false;
        deployed = false;
        //rampServo.setPosition(STORED_RAMP);
    }
}