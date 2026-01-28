package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ArtifactTracker;

import static org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor.GREEN;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Condition;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.ArtifactTracker;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor;

public class SpindexerIsFull implements Condition {

    private ArtifactTracker artifactTracker;
    private Telemetry telemetry;
    ArtifactColor[] currentPattern;
    int numPurple;
    int numGreen;

    public SpindexerIsFull(Telemetry telemetry, ArtifactTracker artifactTracker) {
        this.telemetry = telemetry;
        this.artifactTracker = artifactTracker;
    }

    public boolean check(BlackBoard blackBoard) {
        artifactTracker.getCurrentPatternSnapshot();
        numGreen = 0;
        numPurple = 0;
        return countArtifacts();
    }

    private boolean countArtifacts() {
        for (ArtifactColor artifact : currentPattern) {
            if (artifact == ArtifactColor.PURPLE) {
                numPurple++;
            } else if (artifact == GREEN) {
                numGreen++;
            }
        }

        return (numGreen + numPurple) != 3;
    }
}