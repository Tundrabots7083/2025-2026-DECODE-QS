package org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController;



import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


// TODO Remove opMode and fix camelCase
public class ShooterStorageColorSensorController extends LinearOpMode{
    private NormalizedColorSensor colorSensor;
    private String initPortStorageColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ShooterColorSensor");

    return null;
    }



   public static float gain = 50;



        // Define a variable for our color sensor

        NormalizedRGBA colors;
        @Override
        public void runOpMode() {
// Get the color sensor from hardwareMap
initPortStorageColorSensor(hardwareMap);

// Wait for the Play button to be pressed
            waitForStart();
            final float[] hsvValues = new float[3];
            colors = colorSensor.getNormalizedColors();
            colorSensor.setGain(gain);
             boolean shooterColorSensorIsPurple;
             boolean shooterColorSensorIsGreen;
             boolean shooterColorSensorIsBallPresent;
// While the Op Mode is running, update the telemetry values
            while (opModeIsActive()) {

                colors = colorSensor.getNormalizedColors();



                Color.colorToHSV(colors.toColor(), hsvValues);

               /* telemetry.addLine()
                        .addData("ShooterColorSensorHue", "%.3f", hsvValues[0])
                        .addData("ShooterColorSensorSaturation", "%.3f", hsvValues[1])
                        .addData("ShooterColorSensorValue", "%.3f", hsvValues[2]);
                telemetry.addData("ShooterColorSensorGain", gain);
                 Retained for debug purposes
                 */



                // Comment following 2 lines for debug
                shooterColorSensorIsPurple = shooterColorSensorIsPurple(hsvValues[0]);
                shooterColorSensorIsGreen = shooterColorSensorIsGreen(hsvValues[0]);



                /*shooterColorSensorIsGreen = hsvValues[0] >= 150 && hsvValues[0] <= 160;
                shooterColorSensorIsPurple = hsvValues[0] >= 220 && hsvValues[0] <= 233;
                shooterColorSensorIsBallPresent = (!shooterColorSensorIsPurple && !shooterColorSensorIsGreen);
                telemetry.addData("ShooterColorSensorGreen?",shooterColorSensorIsGreen);
                telemetry.addData("ShooterColorSensorPurple?",shooterColorSensorIsPurple);
                telemetry.addData("ShooterColorSensorBall?",!shooterColorSensorIsBallPresent);
                telemetry.update();
                Retained for debug purposes
                 */
        }

        }

        private boolean shooterColorSensorIsGreen(float Hue){
            return Hue >= 150 && Hue <= 160;
        }
         private boolean shooterColorSensorIsPurple(float Hue){
          return Hue >= 220 && Hue <= 233;
        }


}


