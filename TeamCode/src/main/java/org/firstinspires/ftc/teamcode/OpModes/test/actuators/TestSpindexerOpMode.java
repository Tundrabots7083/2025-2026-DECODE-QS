package org.firstinspires.ftc.teamcode.opModes.test.actuators;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;

import java.util.List;

@Configurable
@Autonomous(name = "Test Spindexer", group = "test")
public class TestSpindexerOpMode extends LinearOpMode {

    public static double targetSpindexPosition = 0;
    public static double targetIntakeVel = 0;
    public static double rampTargetAngle = 0;
    public static double shooterSpeed = 0;

    SpindexerController spindexerController;
    IntakeController intakeController;
    RampController rampController;
    ShooterController shooterController;

    private long lastTime = System.nanoTime();

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    private long count = 0;
    boolean isStuckHasRun = false;
    double currentTarget = 0.0;


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

        joinedTelemetry.addData("TestShooterOpMode", "runOpMode started");
        joinedTelemetry.update();
        waitForStart();

        /// Shooter
        this.spindexerController = SpindexerController.getInstance();

        this.spindexerController.reset();
        this.spindexerController.initialize(hardwareMap, joinedTelemetry);
        /// End Shooter

        /// Intake
        this.intakeController = IntakeController.getInstance();

        this.intakeController.reset();
        this.intakeController.initialize(hardwareMap, joinedTelemetry);
        /// End Intake

        /// Intake
        this.rampController = RampController.getInstance();

        this.rampController.reset();
        this.rampController.initialize(hardwareMap, joinedTelemetry);
        /// End Intake

        /// Intake
        this.shooterController = ShooterController.getInstance();

        this.shooterController.reset();
        this.shooterController.initialize(hardwareMap, joinedTelemetry);
        /// End Intake


        while (opModeIsActive()) {

            // --- Spindexer ---
            // Always feed the desired goal; controller decides whether it is honored
            spindexerController.moveToPosition(targetSpindexPosition);

            // --- Intake ---
            intakeController.spinToTargetVelocity(targetIntakeVel);

            rampController.setTargetPosition(rampTargetAngle);

            shooterController.spinToTargetVelocity(shooterSpeed);

            // --- Telemetry / timing ---
            long currentTime = System.nanoTime();
            double loopTimeMs = (currentTime - lastTime) / 1e6;
            lastTime = currentTime;

            joinedTelemetry.addData("Loop Time (ms)", loopTimeMs);
            joinedTelemetry.update();

            // --- Bulk cache clear ---
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        } // end while loop
    } // end runOpMode
} // end class

