package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

public class SwitchSpindexerState implements ActionFunction {

    Telemetry telemetry;
    Status status;
    boolean hasSpun = false;


    public SwitchSpindexerState(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoard blackBoard) {

        hasSpun = !hasSpun;

        blackBoard.setValue("SpindexerHasSpun", hasSpun);
        status = Status.SUCCESS;
        return status;
    }
}
