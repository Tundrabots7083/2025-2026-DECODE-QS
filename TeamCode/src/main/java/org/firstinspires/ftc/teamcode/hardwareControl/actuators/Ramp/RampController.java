package org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ServoConstants;

@Configurable
public class RampController {

    private ServoImplEx rampServo;

    private double TARGET_POSITION;
    private double MAX_DEGREES;
    private double MIN_DEGREES;

    private final double STORED_RAMP = 0;
    private final double DEPLOYED_RAMP = 100; //degrees at which the ramp is deployed


    private boolean initialized = false;

    // Singleton instance
    private static final RampController INSTANCE = new RampController();
    private Telemetry telemetry;

    private RampController() {}

    public static RampController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants() {
        try {
            Class.forName(RampController.class.getName());
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
        rampServo = hardwareMap.get(ServoImplEx.class, ServoConstants.name);
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

        rampServo.setPosition(servoPosition);
        telemetry.addData("Ramp Target", TARGET_POSITION);
    }

    public double getPosition() {
        return TARGET_POSITION;
    }

    public void deployRamp() {
        if(getPosition() == DEPLOYED_RAMP) {
            return;
        }
        rampServo.setPosition(DEPLOYED_RAMP);
    }

    public void storeRamp() {
        if(getPosition() == STORED_RAMP) {
            return;
        }
        rampServo.setPosition(STORED_RAMP);
    }


    public void update() {}

    public void reset() {
        if (!initialized) return;
        rampServo.setPosition(STORED_RAMP);
    }
}
