package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Condition;

public class HasTriggerNotTripped implements Condition {

    private Telemetry telemetry;

    public HasTriggerNotTripped(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean check(BlackBoard blackBoard) {

        if (blackBoard.getValue("wasTriggerTripped") != null) {
            boolean wasTriggerTripped = (boolean) blackBoard.getValue("wasTriggerTripped");
            return !wasTriggerTripped;
        } else {
            return true;
        }
    }
}