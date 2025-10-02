package org.firstinspires.ftc.teamcode.reference.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

public class Action extends Node {
    private ActionFunction actionFunction;
    protected Telemetry telemetry;


    public Action(ActionFunction actionFunction, Telemetry telemetry) {
        this.actionFunction = actionFunction;
        this.telemetry = telemetry;
    }

    @Override
    public Status execute(BlackBoard blackBoard) {
        return actionFunction.perform(blackBoard);
    }
}
