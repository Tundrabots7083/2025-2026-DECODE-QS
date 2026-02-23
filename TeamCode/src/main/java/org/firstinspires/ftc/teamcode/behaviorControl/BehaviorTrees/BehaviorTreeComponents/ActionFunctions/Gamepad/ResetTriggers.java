package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

public class ResetTriggers implements ActionFunction {

    Telemetry telemetry;
    boolean wasTriggerTripped;

    public ResetTriggers(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoard blackBoard) {

        blackBoard.setValue("wasTriggerTripped", false);
        return Status.SUCCESS;
    }
}