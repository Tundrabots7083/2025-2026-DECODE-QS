package org.firstinspires.ftc.teamcode.reference.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

public abstract class Node {
    public abstract Status execute(BlackBoard globalStore);
}