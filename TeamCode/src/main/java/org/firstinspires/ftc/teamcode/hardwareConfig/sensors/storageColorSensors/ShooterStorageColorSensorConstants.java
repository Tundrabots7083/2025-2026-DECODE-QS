package org.firstinspires.ftc.teamcode.hardwareConfig.sensors.storageColorSensors;




import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ColorDistanceSensorConstants;

/*
        Storage color sensor are named are as followed

                                     ^
                                     I
                              (Front of robot)

                        [ShooterStorageColorSensor]
              [PortStorageColorSensor] [StarboardStorageColorSensor]
 */
public class ShooterStorageColorSensorConstants {
    static {
        ColorDistanceSensorConstants.name = "ShooterStorageColorSensor";
        ColorDistanceSensorConstants.gain = 50;
        ColorDistanceSensorConstants.maxGreen = 160;
        ColorDistanceSensorConstants.minGreen = 150;
        ColorDistanceSensorConstants.maxPurple = 220;
        ColorDistanceSensorConstants.minPurple = 233;

    }
}





