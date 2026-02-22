package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;

public class SpinSpindexerToShoot implements ActionFunction {

    Telemetry telemetry;

    SpindexerController spindexerController;

    Status lastStatus = Status.FAILURE;

    public SpinSpindexerToShoot(Telemetry telemetry, SpindexerController spindexerController) {
        this.telemetry = telemetry;
        this.spindexerController = spindexerController;
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

//        if (blackBoard.getValue("SpindexerPower") != null) {
//            spindexerController.moveToPosition(spindexerController.getTargetPosition() + 360);
//            spindexerController.testSpindexer((double) blackBoard.getValue("SpindexerPower"));
//            lastStatus = Status.SUCCESS;
//        }

        spindexerController.testSpindexer(0.3);
        spindexerController.moveToPosition(spindexerController.getTargetPosition() + 358);
        lastStatus = Status.SUCCESS;
        return Status.SUCCESS;
    }
}
