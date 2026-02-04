package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Condition;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;

public class IsTriggerPulled implements Condition {

    private Telemetry telemetry;

    public IsTriggerPulled(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean check(BlackBoard blackBoard) {

        if (blackBoard.getValue("gamepad1Delta") != null) {
            GamepadDelta gamepad1 = (GamepadDelta) blackBoard.getValue("gamepad1Delta");
            return gamepad1.rightTriggerPulling || gamepad1.leftTriggerPulling;
        } else {
            return false;
        }
    }
}