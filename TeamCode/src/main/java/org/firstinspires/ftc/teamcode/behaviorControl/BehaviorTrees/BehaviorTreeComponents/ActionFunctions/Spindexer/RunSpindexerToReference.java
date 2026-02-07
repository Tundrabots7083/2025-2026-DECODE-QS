package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;

public class RunSpindexerToReference implements ActionFunction {

    Telemetry telemetry;

    SpindexerController spindexerController;
    private Status lastStatus = Status.FAILURE;

    public RunSpindexerToReference(Telemetry telemetry, SpindexerController spindexerController) {
        this.telemetry = telemetry;
        this.spindexerController = spindexerController;
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        boolean switchState = (boolean) blackBoard.getValue("LimitSwitchState");

        // switch is pressed if value returned is LOW or false.
        if (!switchState) {
            spindexerController.spinSlowly();
            return Status.RUNNING;
        } else {
            spindexerController.stop();
            spindexerController.hardwareReset();
            lastStatus = Status.SUCCESS;
            return lastStatus;
        }
    }
}