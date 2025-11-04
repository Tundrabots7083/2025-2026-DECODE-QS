package org.firstinspires.ftc.teamcode.hardwareConfig.sensors.storageColorSensors;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ColorDistanceSensorConstants;
import com.qualcomm.hardware.rev.RevColorSensorV3;

/*
        Storage color sensor are named are as followed

                                     ^
                                     I
                              (Front of robot)

                        [ShooterStorageColorSensor]
              [PortStorageColorSensor] [StarboardStorageColorSensor]
 */
public class StarboardStorageColorSensorConstants {
{
    ColorDistanceSensorConstants.name = "StarboardStorageColorSensor";
    ColorDistanceSensorConstants.distanceUnit = DistanceUnit.MM;
}
}
