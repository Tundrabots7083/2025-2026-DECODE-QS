package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;

public class SpinToZeroPosition implements ActionFunction {

    Telemetry telemetry;
    SpindexerController spindexerController;
    Status status;
    Status lastStatus = Status.FAILURE;

    private boolean hasRun = false;
    private double currentPosition;
    private double targetPosition = 0;

    public SpinToZeroPosition(Telemetry telemetry, SpindexerController spindexerController) {
        this.telemetry = telemetry;
        this.spindexerController = spindexerController;
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        if (!hasRun) {
            currentPosition = spindexerController.getPosition();

            if (currentPosition > 240) {
                spindexerController.moveToPosition(360);
            } else if (currentPosition > 120) {
                spindexerController.moveToPosition(240);
            } else {
                spindexerController.moveToPosition(120);
            }

            hasRun = true;
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
