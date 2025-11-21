package org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.storageColorSensors.StarboardStorageColorSensorConstants;


public class StarboardStorageColorSensorController {
    private boolean initialized = false;


    // Private static instance (eager initialization)
    private static final StarboardStorageColorSensorController INSTANCE = new StarboardStorageColorSensorController();
    private LinearOpMode opMode;
    private Telemetry telemetry;

    private NormalizedColorSensor colorSensor;

    private static float gain = 50;
    NormalizedRGBA colors;

    // Private constructor to prevent instantiation
    private StarboardStorageColorSensorController() {
        // Initialize hardware, state, or configuration here

    }

    // Public method to access the singleton instance
    public static StarboardStorageColorSensorController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants() {
        try {

            Class.forName(StarboardStorageColorSensorConstants.class.getName());
        } catch (ClassNotFoundException e) {
            //e.printStackTrace();
        }
    }

    // Initialization method — must be called once at the beginning
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        if (initialized) {
            return;
            //throw new IllegalStateException("FrontDistanceSensorController has already been initialized.");
        }
        setupConstants();
        telemetry.addData("StarboardStorageColorSensorConstants.class.getName()", StarboardStorageColorSensorConstants.class.getName());

        this.opMode = opMode;
        this.telemetry = telemetry;

        colorSensor = hardwareMap.get(RevColorSensorV3.class, StarboardStorageColorSensorConstants.name);

        colors = colorSensor.getNormalizedColors();
        colorSensor.setGain(StarboardStorageColorSensorConstants.gain);

        initialized = true;
    }

    public void reset() {

    }


    // Example method
    public String getColor() {
        final float[] hsvValues = new float[3];
        colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        boolean isPurple = colorIsPurple(hsvValues[0]);
        boolean isGreen = colorIsGreen(hsvValues[0]);


        if (isPurple) {
            return "PURPLE";
        } else if (isGreen) {
            return "GREEN";
        } else {
            return "EMPTY";
        }

    }


    public void update() {

    }

    private boolean colorIsGreen(float hue) {
        return hue >= StarboardStorageColorSensorConstants.minGreen && hue <= StarboardStorageColorSensorConstants.maxGreen;
    }

    private boolean colorIsPurple(float hue) {
        return hue >= StarboardStorageColorSensorConstants.minPurple && hue <= StarboardStorageColorSensorConstants.maxPurple;
    }
}

/*usage Example*/
