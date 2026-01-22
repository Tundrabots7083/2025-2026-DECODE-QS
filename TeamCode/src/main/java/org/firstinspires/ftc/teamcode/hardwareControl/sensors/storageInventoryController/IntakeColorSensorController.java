package org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.colorSensor.IntakeColorSensorConstants;

public class IntakeColorSensorController {
    private boolean initialized = false;

    // Private static instance (eager initialization)
    private static final IntakeColorSensorController INSTANCE = new IntakeColorSensorController();
    private Telemetry telemetry;
    private IntakeColorSensorConstants colorSensorConstants;

    private NormalizedColorSensor colorSensor;

    NormalizedRGBA colors;

    // Private constructor to prevent instantiation
    private IntakeColorSensorController() {
        // Initialize hardware, state, or configuration here

    }

    // Public method to access the singleton instance
    public static IntakeColorSensorController getInstance() {
        return INSTANCE;
    }


    // Initialization method — must be called once at the beginning
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) {
            return;
        }
        setupConstants();
        this.telemetry = telemetry;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, colorSensorConstants.name);

        colors = colorSensor.getNormalizedColors();
        colorSensor.setGain(colorSensorConstants.gain);

        initialized = true;
    }

    private void setupConstants() {
        colorSensorConstants = new IntakeColorSensorConstants();
    }

    public void reset() {
        initialized = false;
    }

    public ArtifactColor getColor() {
        final float[] hsvValues = new float[3];
        colors = colorSensor.getNormalizedColors();


        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addData("Hue Values", hsvValues[0]);
        telemetry.addData("Saturation Values", hsvValues[1]);
        telemetry.addData("Value Values", hsvValues[2]);
        boolean isPurple = colorIsPurple(hsvValues[0]);
        boolean isGreen = colorIsGreen(hsvValues[0]);


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

    private boolean colorIsGreen(float hue) {
        return hue >= colorSensorConstants.minGreen && hue <= colorSensorConstants.maxGreen;
    }

    private boolean colorIsPurple(float hue) {
        return hue >= colorSensorConstants.minPurple && hue <= colorSensorConstants.maxPurple;
    }
}

