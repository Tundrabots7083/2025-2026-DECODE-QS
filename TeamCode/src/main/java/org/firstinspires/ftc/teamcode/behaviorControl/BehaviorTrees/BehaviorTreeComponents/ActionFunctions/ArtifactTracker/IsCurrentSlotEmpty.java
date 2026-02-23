package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ArtifactTracker;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Condition;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor;

public class IsCurrentSlotEmpty implements Condition {

    private Telemetry telemetry;

    public IsCurrentSlotEmpty(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean check(BlackBoard blackBoard) {

        if (blackBoard.getValue("ArtifactColor") != null) {
            telemetry.addLine("HIIIIIIIIIIIIIIIIIIIIIIIIIII");
            return blackBoard.getValue("ArtifactColor") == ArtifactColor.NONE;
        } else {
            telemetry.addLine("HIIIelooooooooooooooooooooooo");
            return false;
        }
    }
}