package org.firstinspires.ftc.teamcode.opModes.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree.FrontZoneScoringBehaviorTree;


@Autonomous(name="BT Specimen Scoring", group="specimens")
public class SpecimenScoringOpMode extends LinearOpMode
{
    FrontZoneScoringBehaviorTree behaviorTree = null;
    private long count =0;


    @Override
    public void runOpMode()
    {

        telemetry.addData("SpecimenScoringOpMode", "runOpMode started");
        telemetry.update();
        initialize(this);
        waitForStart();


        while (opModeIsActive())
        {
            count++;
          //  telemetry.addData("SpecimenScoringOpMode000", "runOpMode while started count: %d", count);
          //  telemetry.update();
            Status result = this.behaviorTree.tick();


            telemetry.addData("SpecimenScoringOpMode", "Behavior tree result: %s",result);
            telemetry.update();


            if(result == Status.SUCCESS){
                telemetry.addData("SpecimenScoringOpMode", "runOpMode success");
                telemetry.update();
                requestOpModeStop();
            }

        }
    }


    private void initialize(LinearOpMode opMode){
        this.behaviorTree = new FrontZoneScoringBehaviorTree(this);
    }


}


