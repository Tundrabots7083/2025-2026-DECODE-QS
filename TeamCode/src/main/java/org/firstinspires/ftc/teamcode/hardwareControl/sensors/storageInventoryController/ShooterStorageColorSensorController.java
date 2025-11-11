package org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController;



import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Configurable

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




                shooterColorSensorIsPurple = shooterColorSensorIsPurple(hsvValues[0]);
                shooterColorSensorIsGreen = shooterColorSensorIsGreen(hsvValues[0]);
                  //  isBallPresent = (!isPurple && !isGreen);


                /* telemetry.addData("ShooterColorSensorGreen?",isGreen);
                telemetry.addData("ShooterColorSensorPurple?",isPurple);
                telemetry.addData("ShooterColorSensorBall?",!isBallPresent);
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


