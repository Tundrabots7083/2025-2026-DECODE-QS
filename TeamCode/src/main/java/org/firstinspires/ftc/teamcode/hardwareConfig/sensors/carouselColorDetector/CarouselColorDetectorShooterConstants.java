package org.firstinspires.ftc.teamcode.hardwareConfig.sensors.carouselColorDetector;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.DistanceSensorConstants;

public class CarouselColorDetectorShooterConstants {
    static {
        DistanceSensorConstants.name="shooterColorSensor";
        DistanceSensorConstants.maxDistance=200;    //cm --to be set later from the specs
        DistanceSensorConstants.minDistance=2;  //cm --to be set later from the specs
        DistanceSensorConstants.distanceUnit = DistanceUnit.MM;
    }
}
