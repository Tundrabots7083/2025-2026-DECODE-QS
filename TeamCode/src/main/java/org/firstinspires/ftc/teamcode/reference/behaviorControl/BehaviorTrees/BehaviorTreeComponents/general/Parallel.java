package org.firstinspires.ftc.teamcode.reference.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

import java.util.ArrayList;
import java.util.List;

class Parallel extends org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node {
    private final List<org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node> children;

    Parallel(List<org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node> children) {
        this.children = children;
    }

    @Override
    public org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status execute(BlackBoard globalStore) {
        List<org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status> childStatuses = new ArrayList<>();

        for (Node child : children) {
            org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status status = child.execute(globalStore);
            childStatuses.add(status);
        }

        // Check if any child succeeded
        if (childStatuses.contains(org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.SUCCESS)) {
            return org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.SUCCESS;
        }
        // Check if any child is still running
        else if (childStatuses.contains(org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.RUNNING)) {
            return org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.RUNNING;
        }
        // All children failed
        else {
            return Status.FAILURE;
        }
    }
}
