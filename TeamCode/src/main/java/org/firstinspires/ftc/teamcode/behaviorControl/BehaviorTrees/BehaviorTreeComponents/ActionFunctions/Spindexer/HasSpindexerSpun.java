package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;

public class HasSpindexerSpun implements ActionFunction {

    private Telemetry telemetry;
    SpindexerController spindexerController;
    Status lastStatus = Status.FAILURE;

    public HasSpindexerSpun(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.spindexerController = SpindexerController.getInstance();
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        if (spindexerController.isPastTarget()) {
            spindexerController.stop();
            lastStatus = Status.SUCCESS;
        } else {
            lastStatus = Status.RUNNING;
        }

        return lastStatus;
    }
}