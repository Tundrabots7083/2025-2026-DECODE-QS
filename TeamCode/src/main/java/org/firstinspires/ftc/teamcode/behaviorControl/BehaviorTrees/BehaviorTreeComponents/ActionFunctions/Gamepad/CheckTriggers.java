package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;

public class CheckTriggers implements ActionFunction {

    Telemetry telemetry;
    boolean wasTriggerTripped;

    public CheckTriggers(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoard blackBoard) {

        if (blackBoard.getValue("gamepad1Delta") != null) {
            GamepadDelta gamepad_1_Delta = (GamepadDelta) blackBoard.getValue("gamepad1Delta");
            telemetry.addData("IsTriggerNotPulled", !gamepad_1_Delta.rightTriggerPulling && !gamepad_1_Delta.leftTriggerPulling);
            wasTriggerTripped = gamepad_1_Delta.rightTriggerPulling || gamepad_1_Delta.leftTriggerPulling;
        } else {
            return Status.RUNNING;
        }

        if (wasTriggerTripped) {
            blackBoard.setValue("wasTriggerTripped", wasTriggerTripped);
        }

        return Status.SUCCESS;
    }
}