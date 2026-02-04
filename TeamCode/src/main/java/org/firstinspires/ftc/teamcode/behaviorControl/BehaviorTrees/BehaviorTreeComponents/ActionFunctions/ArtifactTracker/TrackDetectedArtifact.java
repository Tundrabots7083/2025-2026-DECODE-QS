package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ArtifactTracker;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.ArtifactTracker;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor;

public class TrackDetectedArtifact implements ActionFunction {

    Telemetry telemetry;

    SpindexerController spindexerController;
    ArtifactTracker artifactTracker;

    protected Status lastStatus = Status.FAILURE;


    public TrackDetectedArtifact(Telemetry telemetry, SpindexerController spindexerController, ArtifactTracker artifactTracker) {
        this.telemetry = telemetry;
        this.spindexerController = spindexerController;
        this.artifactTracker = artifactTracker;
    }

    public Status perform(BlackBoard blackBoard) {

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        if (blackBoard.getValue("ArtifactColor") != null) {
            ArtifactColor artifactColor = (ArtifactColor) blackBoard.getValue("ArtifactColor");
            int currentSlot = spindexerController.getSlotPosition();

            artifactTracker.setArtifact(currentSlot, artifactColor);
        }

        lastStatus = Status.SUCCESS;
        return Status.SUCCESS;
    }
}