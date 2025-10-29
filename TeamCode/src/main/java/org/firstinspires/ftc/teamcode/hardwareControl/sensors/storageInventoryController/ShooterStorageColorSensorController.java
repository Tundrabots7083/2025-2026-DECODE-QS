package org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterStorageColorSensorController {


    private RevColorSensorV3 colorSensor;
    private void shooterStorageColorSensorController (HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "shooterStorageColorSensor");
        colorSensor.enableLed(true);
        colorSensor.status();
    }

}