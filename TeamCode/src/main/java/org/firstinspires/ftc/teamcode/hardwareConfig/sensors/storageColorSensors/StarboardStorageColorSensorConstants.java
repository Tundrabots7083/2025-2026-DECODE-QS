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
public class StarboardStorageColorSensorConstants {

 public static String name = "StarboardStorageColorSensor";
 public static float gain = 50;
 public static double maxGreen = 160;
 public static double minGreen = 150;
 public static double maxPurple = 233;
 public static double minPurple = 220;

}

