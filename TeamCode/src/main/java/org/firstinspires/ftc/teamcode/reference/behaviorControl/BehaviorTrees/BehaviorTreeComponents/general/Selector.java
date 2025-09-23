package org.firstinspires.ftc.teamcode.reference.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

import java.util.List;

public class Selector extends org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node {
    private List<org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node> children;
    protected Telemetry telemetry;

    public Selector(List<org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node> children, Telemetry telemetry) {
        this.children = children;
        this.telemetry = telemetry;

    }

    @Override
    public org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status execute(BlackBoard globalStore) {
        for (Node child : children) {
            org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status status = child.execute(globalStore);
            telemetry.addData("Selector", "Selector status: %b",status == org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.SUCCESS);
            telemetry.update();
            if (status == org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.SUCCESS) {
                return org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.SUCCESS;
            } else if (status == org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.RUNNING) {
                return org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.RUNNING;
            }
        }
        return Status.FAILURE;
    }
}
