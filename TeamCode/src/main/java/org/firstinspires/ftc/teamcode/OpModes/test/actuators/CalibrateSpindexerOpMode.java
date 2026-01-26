package org.firstinspires.ftc.teamcode.opModes.test.actuators;


import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree.SpindexerTestBehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;


@Autonomous(name = "Calibrate Spindexer BehaviorTree", group = "test")
public class CalibrateSpindexerOpMode extends LinearOpMode {
    SpindexerTestBehaviorTree behaviorTree = null;


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
        this.behaviorTree = new SpindexerTestBehaviorTree(opMode, joinedTelemetry);
    }


}


