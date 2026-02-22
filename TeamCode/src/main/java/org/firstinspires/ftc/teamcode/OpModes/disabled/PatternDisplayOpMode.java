package org.firstinspires.ftc.teamcode.opModes.disabled;


import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree.GenericTestBehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

@Disabled
@Autonomous(name = "Display Motif BehaviorTree", group = "test")
public class PatternDisplayOpMode extends LinearOpMode {
    GenericTestBehaviorTree behaviorTree = null;


    public JoinedTelemetry joinedTelemetry;


    @Override
    public void runOpMode() {

        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        joinedTelemetry.update();
        initialize(this);
        waitForStart();


        while (opModeIsActive()) {
            Status result = this.behaviorTree.tick();
            telemetry.update();


            if (result == Status.SUCCESS) {
                requestOpModeStop();
            }

        }
    }


    private void initialize(LinearOpMode opMode) {
        this.behaviorTree = new GenericTestBehaviorTree(opMode, joinedTelemetry);
    }


}


