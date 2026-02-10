package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;

public class SwitchToShootCoordinates implements ActionFunction {

    Telemetry telemetry;
    SpindexerController spindexerController;
    Status lastStatus = Status.FAILURE;

    double intakeDegreeOffset = -50.0;

    public SwitchToShootCoordinates(Telemetry telemetry, SpindexerController spindexerController) {
        this.telemetry = telemetry;
        this.spindexerController = spindexerController;
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        spindexerController.setDegreeOffset(intakeDegreeOffset);

        if (spindexerController.isOnTarget()) {
            lastStatus = Status.SUCCESS;
            return Status.SUCCESS;
        } else {
            return Status.RUNNING;
        }

    }
}