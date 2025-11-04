package org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.storageColorSensors.StarboardStorageColorSensorConstants;

@TeleOp(name = "Test color sensor")
public class StorageColorSensorsController extends LinearOpMode{
    private RevColorSensorV3 colorSensor;
    private String initPortStorageColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ShooterColorSensor");
        colorSensor.enableLed(false);
        colorSensor.status();
    return colorSensor.status();
    }





        // Define a variable for our color sensor


        @Override
        public void runOpMode() {
// Get the color sensor from hardwareMap
initPortStorageColorSensor(hardwareMap);

// Wait for the Play button to be pressed
            waitForStart();

// While the Op Mode is running, update the telemetry values
            while (opModeIsActive()) {
                colorSensor.enableLed(true);
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                telemetry.addData("Red", colors.red);
                telemetry.addData("Green", colors.green);
                telemetry.addData("Blue", colors.blue);
                telemetry.addData("Status", colorSensor.status());
                telemetry.update();

            }
        }
}

