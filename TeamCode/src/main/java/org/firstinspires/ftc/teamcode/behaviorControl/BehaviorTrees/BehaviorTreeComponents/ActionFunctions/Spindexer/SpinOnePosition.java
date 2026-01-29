package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;

public class SpinOnePosition implements ActionFunction {

    Telemetry telemetry;
    SpindexerController spindexerController;
    Status status;
    Status lastStatus = Status.FAILURE;
    boolean isAutonomous;
    boolean hasSpun = false;

    private double currentTarget;
    private double targetPosition;

    public SpinOnePosition(Telemetry telemetry, SpindexerController spindexerController) {
        this.telemetry = telemetry;
        this.spindexerController = spindexerController;
    }

    public Status perform(BlackBoard blackBoard) {
        isAutonomous = (boolean) blackBoard.getValue("isAutonomous");
        if (lastStatus == Status.SUCCESS && (isAutonomous)) {
            return lastStatus;
        }

        if (!hasSpun) {
            currentTarget = spindexerController.getTargetPosition();
            targetPosition = currentTarget + 120;
            spindexerController.moveToPosition(targetPosition);
            hasSpun = true;
        }

        if (!spindexerController.isOnTarget()) {
            status = Status.RUNNING;
        } else {
            hasSpun = false;
            status = Status.SUCCESS;
        }

        lastStatus = status;
        return status;
    }
}
