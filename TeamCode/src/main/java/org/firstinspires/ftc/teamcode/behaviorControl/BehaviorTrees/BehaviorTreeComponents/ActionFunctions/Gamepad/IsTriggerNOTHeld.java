package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Condition;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.gamepad.GamepadSnapshot;

public class IsTriggerNOTHeld implements Condition {

    private Telemetry telemetry;

    public IsTriggerNOTHeld(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean check(BlackBoard blackBoard) {

        if (blackBoard.getValue("gamepad1Snapshot") != null) {
            GamepadSnapshot gamepad1 = (GamepadSnapshot) blackBoard.getValue("gamepad1Snapshot");
            return !(gamepad1.rightTrigger >= 0.5) && !(gamepad1.leftTrigger >= 0.5);
        } else {
            return false;
        }
    }
}