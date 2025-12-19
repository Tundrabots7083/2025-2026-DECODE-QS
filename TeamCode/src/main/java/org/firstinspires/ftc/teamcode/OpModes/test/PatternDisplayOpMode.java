package org.firstinspires.ftc.teamcode.opModes.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree.DisplayTestBehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;


@Autonomous(name="Display Motif BehaviorTree", group="test")
public class PatternDisplayOpMode extends LinearOpMode
{
    DisplayTestBehaviorTree behaviorTree = null;
    private long count =0;
    


    @Override
    public void runOpMode()
    {

        telemetry.addData("Display Motif OpMode", "runOpMode started");
        telemetry.update();
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
        this.behaviorTree = new DisplayTestBehaviorTree(opMode);
    }


}


