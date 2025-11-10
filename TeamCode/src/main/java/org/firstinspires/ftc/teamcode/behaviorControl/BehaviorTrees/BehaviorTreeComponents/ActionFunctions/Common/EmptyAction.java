package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

/// This action is to be used as a place holder when testing and debugging
public class EmptyAction implements ActionFunction {
    Telemetry telemetry;
    protected LinearOpMode opMode;


    public EmptyAction (Telemetry telemetry, LinearOpMode opMode) {
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.init();
    }

    private void init(){


    }
    @Override
    public Status perform(BlackBoard blackBoard) {
        Status status;

        telemetry.addData("EmptyAction", "perform start");
        telemetry.update();



        status=Status.SUCCESS;



        telemetry.addData("EmptyAction", "perform finish");
        telemetry.update();
        return status;
    }
}

