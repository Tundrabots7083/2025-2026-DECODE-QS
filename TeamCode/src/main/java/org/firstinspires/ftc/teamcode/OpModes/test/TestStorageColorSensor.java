package org.firstinspires.ftc.teamcode.OpModes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.PortStorageColorSensorController;

import java.util.List;

@Configurable
@Autonomous(name = "Test ColorSensors", group = "test")
public class TestStorageColorSensor extends LinearOpMode  {

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    PortStorageColorSensorController portStorageColorSensorController;
    public List<LynxModule> allHubs;
    public void runOpMode() {

        // Enable bulk reads. This is almost always the "correct" answer, and can speed up loop
        // times. We will be managing the bulk read caches manually, which requires each OpMode
        // to clear the cache at the start of each loop.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        /// PortStorageColorSensor
        this.portStorageColorSensorController = PortStorageColorSensorController.getInstance();

        this.portStorageColorSensorController.reset();
        this.portStorageColorSensorController.initialize(hardwareMap, joinedTelemetry, this);
        /// End PortStorageColorSensor

        while (opModeIsActive()) {
            String color = portStorageColorSensorController.getColor();
            joinedTelemetry.addData("Color", color);
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }

    }
}
