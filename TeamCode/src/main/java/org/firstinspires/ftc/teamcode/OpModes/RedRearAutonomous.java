package org.firstinspires.ftc.teamcode.opModes;


import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree.SampleAutoBehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;


@Autonomous(name = "Red Rear Auto", group = "test")
public class RedRearAutonomous extends LinearOpMode {
    SampleAutoBehaviorTree mainBehaviorTree = null;


    public JoinedTelemetry joinedTelemetry;


    @Override
    public void runOpMode() {

        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        initialize(this);

        joinedTelemetry.addLine("Initialization Complete");
        joinedTelemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            Status result = this.mainBehaviorTree.tick();
            joinedTelemetry.update();

        }
    }


    private void initialize(LinearOpMode opMode) {
        this.mainBehaviorTree = new SampleAutoBehaviorTree(opMode, joinedTelemetry);
    }


}


