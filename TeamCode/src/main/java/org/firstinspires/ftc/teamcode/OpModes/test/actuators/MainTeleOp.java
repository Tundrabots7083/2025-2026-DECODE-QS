package org.firstinspires.ftc.teamcode.opModes.test.actuators;


import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree.TeleOp.TeleOpBehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree.TeleOp.TeleOpInitializeBehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;


@TeleOp(name = "Main TeleOp", group = "test")
public class MainTeleOp extends LinearOpMode {
    TeleOpInitializeBehaviorTree initBehaviorTree = null;
    TeleOpBehaviorTree mainBehaviorTree = null;

    boolean isBotInitialized = false;


    public JoinedTelemetry joinedTelemetry;


    @Override
    public void runOpMode() {

        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        initialize(this);

        while (!isBotInitialized) {
            Status initStatus = initBehaviorTree.tick();
            isBotInitialized = (initStatus == Status.SUCCESS);

            if (initStatus == Status.FAILURE) {
                requestOpModeStop();
            }
        }

        joinedTelemetry.addLine("Initialization Complete");
        joinedTelemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            Status result = this.mainBehaviorTree.tick();
            joinedTelemetry.update();


            if (result == Status.FAILURE) {
                requestOpModeStop();
            }

        }
    }


    private void initialize(LinearOpMode opMode) {
        this.initBehaviorTree = new TeleOpInitializeBehaviorTree(opMode, joinedTelemetry);
        this.mainBehaviorTree = new TeleOpBehaviorTree(opMode, joinedTelemetry);
    }


}


