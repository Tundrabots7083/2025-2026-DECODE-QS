package org.firstinspires.ftc.teamcode.opModes.test.actuators;


import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree.AutoIntakeTestBehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

@Disabled
@Autonomous(name = "Test Auto Intake BehaviorTree", group = "test")
public class TestAutoIntakeOpMode extends LinearOpMode {
    AutoIntakeTestBehaviorTree behaviorTree = null;


    public JoinedTelemetry joinedTelemetry;
    private long lastTime = System.nanoTime();


    @Override
    public void runOpMode() {

        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        joinedTelemetry.update();
        initialize(this);
        waitForStart();


        while (opModeIsActive()) {
            Status result = this.behaviorTree.tick();
            long currentTime = System.nanoTime();
            double loopTimeMs = (currentTime - lastTime) / 1e6;
            lastTime = currentTime;

            joinedTelemetry.addData("Loop Time (ms)", loopTimeMs);
            telemetry.update();


            if (result == Status.SUCCESS) {
                requestOpModeStop();
            }

        }
    }


    private void initialize(LinearOpMode opMode) {
        this.behaviorTree = new AutoIntakeTestBehaviorTree(opMode, joinedTelemetry);
    }


}


