package org.firstinspires.ftc.teamcode.opModes.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree.DetectPoseBehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;


@Autonomous(name="BT AprilTag", group="test")
public class DetectBotPoseUsingAprilTag extends LinearOpMode
{
    DetectPoseBehaviorTree behaviorTree = null;
    private long count =0;

//     JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry);

    @Override
    public void runOpMode()
    {

//        joinedTelemetry.addData("DecodeScoringOpMode", "runOpMode started");
//        joinedTelemetry.update();
        initialize(this);
        waitForStart();


        while (opModeIsActive())
        {
            count++;
          //  joinedTelemetry.addData("DecodeScoringOpMode000", "runOpMode while started count: %d", count);
          //  joinedTelemetry.update();
            Status result = this.behaviorTree.tick();


//            joinedTelemetry.addData("DecodeScoringOpMode", "Behavior tree result: %s",result);
//            joinedTelemetry.update();


            if(result == Status.SUCCESS){
//                requestOpModeStop();
            }

        }
    }


    private void initialize(LinearOpMode opMode){
        this.behaviorTree = new DetectPoseBehaviorTree(opMode);
    }


}


