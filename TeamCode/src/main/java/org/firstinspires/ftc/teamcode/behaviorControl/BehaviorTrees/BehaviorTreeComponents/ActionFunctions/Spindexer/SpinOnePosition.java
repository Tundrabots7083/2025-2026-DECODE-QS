package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;

public class SpinOnePosition implements ActionFunction {

    Telemetry telemetry;

    SpindexerController spindexerController;

    boolean hasRun = false;
    private double currentPosition;
    private double targetPosition;

    public SpinOnePosition(Telemetry telemetry, SpindexerController spindexerController) {
        this.telemetry = telemetry;
        this.spindexerController = spindexerController;
    }

    public Status perform(BlackBoard blackBoard) {
        if(!hasRun) {
            currentPosition = spindexerController.getPosition();
            targetPosition = currentPosition + 120;
            spindexerController.moveToPosition(targetPosition);
            hasRun = true;
        }

        if (!spindexerController.isOnTarget()) {
            spindexerController.update();
            return Status.RUNNING;
        } else {
            return Status.SUCCESS;
        }
    }
}
