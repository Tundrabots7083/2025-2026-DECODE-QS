package org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.intakeDistanceSensor.IntakeDistanceSensorConstants;

public class DistanceSensorController {
    private boolean initialized = false;

    // Private static instance (eager initialization)
    private static final DistanceSensorController INSTANCE = new DistanceSensorController();
    private Telemetry telemetry;
    private IntakeDistanceSensorConstants distanceSensorConstants;

    private DistanceSensor distanceSensor;


    // Private constructor to prevent instantiation
    private DistanceSensorController() {
    }

    // Public method to access the singleton instance
    public static DistanceSensorController getInstance() {
        return INSTANCE;
    }


    // Initialization method — must be called once at the beginning
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) {
            return;
        }
        setupConstants();
        this.telemetry = telemetry;

        distanceSensor = hardwareMap.get(DistanceSensor.class, distanceSensorConstants.name);

        initialized = true;
    }

    private void setupConstants() {
        distanceSensorConstants = new IntakeDistanceSensorConstants();
    }

    public void reset() {
        initialized = false;
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }

    public void update() {
    }
}

