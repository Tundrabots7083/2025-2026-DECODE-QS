package org.firstinspires.ftc.teamcode.reference.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

import java.util.List;

public class Sequence extends org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node {
    private List<org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node> children;
    protected Telemetry telemetry;

    public Sequence(List<org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node> children, Telemetry telemetry) {
        this.children = children;
        this.telemetry = telemetry;
    }

    @Override
    public org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status execute(BlackBoard globalStore) {
        for (Node child : children) {
            org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status status = child.execute(globalStore);
          //  opMode.telemetry.addData("Sequence", "Sequence execute result: %b num children = %d",status==Status.SUCCESS, children.stream().count());
          //  opMode.telemetry.update();
            if (status == org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.FAILURE) {
                return org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.FAILURE;
            } else if (status == org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.RUNNING) {
                return org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status.RUNNING;
            }
        }
       // opMode.telemetry.addData("Sequence", "Sequence execute ---------------");
       // opMode.telemetry.update();
        return Status.SUCCESS;
    }
}
