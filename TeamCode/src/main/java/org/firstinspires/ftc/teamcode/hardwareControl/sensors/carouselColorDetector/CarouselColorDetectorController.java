package org.firstinspires.ftc.teamcode.hardwareControl.sensors.carouselColorDetector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.DistanceSensorConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.carouselColorDetector.CarouselColorDetectorPortConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.carouselColorDetector.CarouselColorDetectorShooterConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.carouselColorDetector.CarouselColorDetectorStarboardConstants;


public class CarouselColorDetectorController {
    private boolean initialized = false;


    // Private static instance (eager initialization)
    private static final CarouselColorDetectorController INSTANCE = new CarouselColorDetectorController();
    private LinearOpMode opMode;
    private Telemetry telemetry;

    private DistanceSensor shooterColorSensor;
    private DistanceSensor portColorSensor;
    private DistanceSensor starboardColorSensor;
    private DistanceUnit distanceUnit;


    // Private constructor to prevent instantiation
    private CarouselColorDetectorController() {
        // Initialize hardware, state, or configuration here

    }

    // Public method to access the singleton instance
    public static CarouselColorDetectorController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants(){
        try {
            Class.forName(CarouselColorDetectorShooterConstants.class.getName());
            Class.forName(CarouselColorDetectorPortConstants.class.getName());
            Class.forName(CarouselColorDetectorStarboardConstants.class.getName());
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

        shooterColorSensor = hardwareMap.get(DistanceSensor.class, DistanceSensorConstants.name);
        distanceUnit = DistanceSensorConstants.distanceUnit;

        initialized = true;
    }

    public void reset() {

    }


    // Example method
    public double getCurrentDistance(){
        double distance = shooterColorSensor.getDistance(distanceUnit);

        /// TODO: Add filter to reduce noise
        telemetry.addData("Distance (MM)", distance);
        telemetry.addData("Distance (CM)", shooterColorSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance (IN)", shooterColorSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        return distance;

    }



    public void update(){

    }

}

/*usage Example*/
