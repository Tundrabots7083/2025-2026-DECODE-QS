package org.firstinspires.ftc.teamcode.opModes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Turret.TurretController;

import java.util.List;

@Configurable
@Autonomous(name = "Test Turret", group = "test")
public class TestTurretOpMode extends LinearOpMode {

    TurretController turretController;

    private long lastTime = System.nanoTime();

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    private long count = 0;
    public static double position = 0.0;

    // All lynx module hubs
    public List<LynxModule> allHubs;


    @Override
    public void runOpMode() {

        // Enable bulk reads. This is almost always the "correct" answer, and can speed up loop
        // times. We will be managing the bulk read caches manually, which requires each OpMode
        // to clear the cache at the start of each loop.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        joinedTelemetry.addData("TestTurretOpMode", "initialized");
        joinedTelemetry.update();
        waitForStart();

        /// Color Sensor
        this.turretController = TurretController.getInstance();

        this.turretController.reset();
        this.turretController.initialize(hardwareMap, joinedTelemetry);
        /// End Color Sensor

        while (opModeIsActive()) {

            long currentTime = System.nanoTime();
            double loopTimeMs = (currentTime - lastTime) / 1e6;
            lastTime = currentTime;

            joinedTelemetry.addData("Loop Time (ms)", loopTimeMs);

            turretController.setTargetPosition(position);
            count++;
            joinedTelemetry.update();

            // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
            // as the bulk read caches are being handled manually.
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }

}

