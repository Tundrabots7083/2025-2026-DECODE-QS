package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;


public class PauseAction implements ActionFunction {
    private final long pauseDuration;
    private long startTime;
    private long endTime;
    Telemetry telemetry;
    protected LinearOpMode opMode;
    private boolean hasRun = false;

    protected Status lastStatus = Status.FAILURE;

    public PauseAction (long pauseDuration, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.pauseDuration = pauseDuration;
    }


    @Override
    public Status perform(BlackBoard blackBoard) {
        Status status;


        if(!hasRun) {
            startTime = System.currentTimeMillis();
            endTime = startTime + pauseDuration;
            hasRun = true;
        }

        if (System.currentTimeMillis() < endTime) {
            telemetry.addData("PauseAction", "perform start");
            status = Status.RUNNING;
        } else {
            status = Status.SUCCESS;
            telemetry.addData("PauseAction", "perform finish");
        }

        return status;
    }
}

