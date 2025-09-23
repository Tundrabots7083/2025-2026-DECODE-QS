package org.firstinspires.ftc.teamcode.hardwareConfig.sensors.intakeDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.DistanceSensorConstants;

public class IntakeDistanceSensorConstants {
    static {
        DistanceSensorConstants.name="testDistance";
        DistanceSensorConstants.maxDistance=200;    //cm --to be set later from the specs
        DistanceSensorConstants.minDistance=2;  //cm --to be set later from the specs
        DistanceSensorConstants.distanceUnit = DistanceUnit.MM;
    }
}