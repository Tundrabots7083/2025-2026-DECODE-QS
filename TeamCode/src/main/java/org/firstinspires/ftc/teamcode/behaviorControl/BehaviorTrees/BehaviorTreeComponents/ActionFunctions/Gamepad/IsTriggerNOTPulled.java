package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Condition;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;

public class IsTriggerNOTPulled implements Condition {

    private Telemetry telemetry;

    public IsTriggerNOTPulled(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean check(BlackBoard blackBoard) {

        if (blackBoard.getValue("gamepad1Delta") != null) {
            GamepadDelta gamepad_1_Delta = (GamepadDelta) blackBoard.getValue("ArtifactColor");
            return !gamepad_1_Delta.rightTriggerPulling && !gamepad_1_Delta.leftTriggerPulling;
        } else {
            return false;
        }
    }
}