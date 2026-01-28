package org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.colorSensor.LEFTIntakeColorSensorConstants;

public class LeftIntakeColorSensorController {
    private boolean initialized = false;

    // Private static instance (eager initialization)
    private static final LeftIntakeColorSensorController INSTANCE = new LeftIntakeColorSensorController();
    private Telemetry telemetry;
    private LEFTIntakeColorSensorConstants colorSensorConstants;

    private RevColorSensorV3 colorSensor;

    int red;
    int green;
    int blue;

    // Private constructor to prevent instantiation
    private LeftIntakeColorSensorController() {
        // Initialize hardware, state, or configuration here

    }

    // Public method to access the singleton instance
    public static LeftIntakeColorSensorController getInstance() {
        return INSTANCE;
    }


    // Initialization method — must be called once at the beginning
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) {
            return;
        }
        setupConstants();
        this.telemetry = telemetry;

        colorSensor = hardwareMap.get(RevColorSensorV3.class, colorSensorConstants.name);

        colorSensor.setGain(colorSensorConstants.gain);

        initialized = true;
    }

    private void setupConstants() {
        colorSensorConstants = new LEFTIntakeColorSensorConstants();
    }

    public void reset() {
        initialized = false;
    }

    public ArtifactColor getColor() {
        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();

        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);
        boolean isGreen = colorIsGreen();
        boolean isPurple = colorIsPurple();


        if (isPurple) {
            return ArtifactColor.PURPLE;
        } else if (isGreen) {
            return ArtifactColor.GREEN;
        } else {
            return ArtifactColor.NONE;
        }

    }

    public void update() {
    }

    private boolean colorIsGreen() {
        return green > colorSensorConstants.redScalar * red
                && green >= colorSensorConstants.blueScalar * blue
                && green > colorSensorConstants.minGreen;
    }

    private boolean colorIsPurple() {
        int maxRedOrBlue = Math.max(red, blue);
        int minRedOrBlue = Math.min(red, blue);

        return maxRedOrBlue > 40
                && minRedOrBlue >= 0.5 * maxRedOrBlue
                && green < 0.7 * maxRedOrBlue;
    }
}

