package org.firstinspires.ftc.teamcode.reference.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Condition;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

public class Conditional extends Node {
    private org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Condition condition;

    public Conditional(Condition condition) {
        this.condition = condition;
    }

    @Override
    public org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status execute(BlackBoard blackBoard) {
        boolean result = condition.check(blackBoard);
        if (result) {
            return org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.SUCCESS;
        } else {
            return Status.FAILURE;
        }

        // return condition.check() ? Status.SUCCESS : Status.FAILURE;
    }
}
