package org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ColorDistanceSensorConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.storageColorSensors.PortStorageColorSensorConstants;

public class PortStorageColorSensorController {
    private boolean initialized = false;


    // Private static instance (eager initialization)
    private static final PortStorageColorSensorController INSTANCE = new PortStorageColorSensorController();
    private LinearOpMode opMode;
    private Telemetry telemetry;

    private NormalizedColorSensor colorSensor;

    private static float gain = 50;
    NormalizedRGBA colors;

    // Private constructor to prevent instantiation
    private PortStorageColorSensorController() {
        // Initialize hardware, state, or configuration here

    }

    // Public method to access the singleton instance
    public static PortStorageColorSensorController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants(){
        try {
            Class.forName(PortStorageColorSensorConstants.class.getName());
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
        this.opMode = opMode;
        this.telemetry  = telemetry;

        colorSensor = hardwareMap.get(RevColorSensorV3.class, ColorDistanceSensorConstants.name);

        colors = colorSensor.getNormalizedColors();
        colorSensor.setGain(gain);

        initialized = true;
    }

    public void reset() {

    }


    // Example method
    public String getColor(){
        final float[] hsvValues = new float[3];
        colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);

        boolean isPurple = shooterColorSensorIsPurple(hsvValues[0]);
        boolean isGreen = shooterColorSensorIsGreen(hsvValues[0]);


      if(isPurple){
        return "PURPLE";
      } else if(isGreen){
          return "GREEN";
      } else {
          return "EMPTY";
      }

      }


        public void update(){

    }
    private boolean shooterColorSensorIsGreen(float hue){
        return hue >= ColorDistanceSensorConstants.minGreen && hue <= ColorDistanceSensorConstants.maxGreen;
    }
    private boolean shooterColorSensorIsPurple(float hue){
        return hue >= ColorDistanceSensorConstants.minPurple && hue <= ColorDistanceSensorConstants.maxPurple;
    }
}

/*usage Example*/
