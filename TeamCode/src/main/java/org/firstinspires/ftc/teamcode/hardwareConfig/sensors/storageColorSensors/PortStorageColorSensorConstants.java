package org.firstinspires.ftc.teamcode.hardwareConfig.sensors.storageColorSensors;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ColorDistanceSensorConstants;

/*
        Storage color sensor are named are as followed

                                     ^
                                     I
                              (Front of robot)

                        [ShooterStorageColorSensor]
              [PortStorageColorSensor] [StarboardStorageColorSensor]
 */

public  class PortStorageColorSensorConstants {
        public static String name = "PortStorageColorSensor";
        public static float gain = 50;
        public static double maxGreen = 180;
        public static double minGreen = 120;
        public static double maxPurple = 240;
        public static double minPurple = 220;




}
