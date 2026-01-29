package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

public class SetTeleOp implements ActionFunction {

    Telemetry telemetry;

    public SetTeleOp(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoard blackBoard) {
        blackBoard.setValue("isAutonomous", false);
        return Status.SUCCESS;
    }
}