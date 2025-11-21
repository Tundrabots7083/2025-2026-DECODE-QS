package org.firstinspires.ftc.teamcode.OpModes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ShooterStorageColorSensorController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.StarboardStorageColorSensorController;

import java.util.List;

@Configurable
@Autonomous(name = "Test ColorSensors", group = "test")
public class TestStorageColorSensor extends LinearOpMode {

    public List<LynxModule> allHubs;
    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    // PortStorageColorSensorController portStorageColorSensorController;
    ShooterStorageColorSensorController shooterStorageColorSensorController;
    StarboardStorageColorSensorController starboardStorageColorSensorController;

    public void runOpMode() {

        // Enable bulk reads. This is almost always the "correct" answer, and can speed up loop
        // times. We will be managing the bulk read caches manually, which requires each OpMode
        // to clear the cache at the start of each loop.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        /// PortStorageColorSensor
//        this.portStorageColorSensorController = PortStorageColorSensorController.getInstance();
//
//        this.portStorageColorSensorController.reset();
//        this.portStorageColorSensorController.initialize(hardwareMap, joinedTelemetry, this);
        /// End PortStorageColorSensor

        /// ShooterStorageColorSensor
        this.shooterStorageColorSensorController = ShooterStorageColorSensorController.getInstance();

        this.shooterStorageColorSensorController.reset();
        this.shooterStorageColorSensorController.initialize(hardwareMap, joinedTelemetry, this);
        /// End ShooterStorageColorSensor

        /// StarboardStorageColorSensor
        this.starboardStorageColorSensorController = StarboardStorageColorSensorController.getInstance();

        this.starboardStorageColorSensorController.reset();
        this.starboardStorageColorSensorController.initialize(hardwareMap, joinedTelemetry, this);
        /// End StarboardStorageColorSensor


        waitForStart();

        while (opModeIsActive()) {
//              TODO add new hardware to test port
//            String portColor = portStorageColorSensorController.getColor();
//           telemetry.addData("portColor", portColor);
//
//           joinedTelemetry.addData("portColor", portColor);
//
            String shooterColor = shooterStorageColorSensorController.getColor();
            telemetry.addData("shooterColor", shooterColor);

            joinedTelemetry.addData("shooterColor", shooterColor);

            String starboardColor = starboardStorageColorSensorController.getColor();
            telemetry.addData("starboardColor", starboardColor);
            joinedTelemetry.addData("starboardColor", starboardColor);
            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }

    }
}
