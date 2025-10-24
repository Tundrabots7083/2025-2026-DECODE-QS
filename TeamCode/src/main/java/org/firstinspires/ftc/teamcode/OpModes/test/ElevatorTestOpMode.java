package org.firstinspires.ftc.teamcode.OpModes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareControl.actuators.elevator.ElevatorController;

import java.util.List;

@Configurable
@Autonomous(name = "Test Elevator", group = "test")
public class ElevatorTestOpMode extends LinearOpMode {
    ElevatorController elevatorController;
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

        waitForStart();

        this.elevatorController = ElevatorController.getInstance();

        while (opModeIsActive()) {
            this.elevatorController.moveUp();
            this.elevatorController.moveDown();
        }
    }
}