package org.firstinspires.ftc.teamcode.opModes.test;


import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree.DisplayTestBehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;


@Autonomous(name="Display Motif BehaviorTree", group="test")
public class PatternDisplayOpMode extends LinearOpMode
{
    DisplayTestBehaviorTree behaviorTree = null;
    private long count =0;


    public JoinedTelemetry joinedTelemetry;


    @Override
    public void runOpMode()
    {

        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        joinedTelemetry.addData("Display Motif OpMode", "runOpMode started");
        joinedTelemetry.update();
        initialize(this);
        waitForStart();


        while (opModeIsActive())
        {
            count++;
            Status result = this.behaviorTree.tick();



            if(result == Status.SUCCESS){
                requestOpModeStop();
            }

        }
    }


    private void initialize(LinearOpMode opMode){
        this.behaviorTree = new DisplayTestBehaviorTree(opMode, joinedTelemetry);
    }


}


