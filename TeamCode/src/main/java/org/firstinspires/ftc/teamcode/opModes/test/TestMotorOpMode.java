package org.firstinspires.ftc.teamcode.opModes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;

import java.util.List;

@Configurable
@Autonomous(name = "Test Shooter", group = "test")
public class TestMotorOpMode extends LinearOpMode {

    public static double velocityTargetPosition = 50;
    ShooterController shooterController;

    private long lastTime = System.nanoTime();

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    private long count = 0;

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

        joinedTelemetry.addData("TestMotorOpMode", "runOpMode started");
        joinedTelemetry.update();
        waitForStart();

        /// Shooter
        this.shooterController = ShooterController.getInstance();

        this.shooterController.reset();
        this.shooterController.initialize(hardwareMap, joinedTelemetry, this);
        /// End Shooter

        while (opModeIsActive()) {
            shooterController.spinToTargetVelocity(velocityTargetPosition);

            long currentTime = System.nanoTime();
            double loopTimeMs = (currentTime - lastTime) / 1e6;
            lastTime = currentTime;

            joinedTelemetry.addData("Loop Time (ms)", loopTimeMs);

            count++;
            joinedTelemetry.addData("Test Shoulder", "runOpMode while started count: %d", count);
            joinedTelemetry.addData("CurrentFrontVelocity", shooterController.getFrontCurrentVelocity());
            joinedTelemetry.addData("CurrentRearVelocity", shooterController.getRearCurrentVelocity());
            joinedTelemetry.addData("TargetVelocity", velocityTargetPosition);
            joinedTelemetry.addData("Encoder Position", shooterController.getFrontCurrentPosition());
            joinedTelemetry.update();

            // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
            // as the bulk read caches are being handled manually.
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }

}

