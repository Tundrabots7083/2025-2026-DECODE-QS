package org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.storageColorSensors.StarboardStorageColorSensorConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.storageColorSensors.PortStorageColorSensorConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.storageColorSensors.ShooterStorageColorSensorConstants;
public class StorageColorSensorsController {
    private RevColorSensorV3 colorSensor;
    private void initPortStorageColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.enableLed(true);
        colorSensor.status();
    }
}

