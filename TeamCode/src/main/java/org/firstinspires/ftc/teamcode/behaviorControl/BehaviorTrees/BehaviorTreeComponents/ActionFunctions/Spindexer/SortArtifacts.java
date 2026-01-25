package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;

public class SortArtifacts implements ActionFunction {

    Telemetry telemetry;
    SpindexerController spindexerController;
    Status status;
    Status lastStatus = Status.FAILURE;

    private double currentPosition;
    private double targetPosition;

    public SortArtifacts(Telemetry telemetry, SpindexerController spindexerController) {
        this.telemetry = telemetry;
        this.spindexerController = spindexerController;
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        if (blackBoard.getValue("spindexerTurnsToPattern") != null) {
            if ((int) blackBoard.getValue("spindexerTurnsToPattern") == 1) {
                currentPosition = spindexerController.getPosition();
                targetPosition = currentPosition + 120;
                spindexerController.moveToPosition(targetPosition);
            } else if ((int) blackBoard.getValue("spindexerTurnsToPattern") == 2) {
                currentPosition = spindexerController.getPosition();
                targetPosition = currentPosition + 240;
                spindexerController.moveToPosition(targetPosition);
            }
        } else {
            status = Status.FAILURE;
        }


        if (!spindexerController.isOnTarget()) {
            status = Status.RUNNING;
        } else {
            status = Status.SUCCESS;
        }

        lastStatus = status;
        return status;
    }
}
