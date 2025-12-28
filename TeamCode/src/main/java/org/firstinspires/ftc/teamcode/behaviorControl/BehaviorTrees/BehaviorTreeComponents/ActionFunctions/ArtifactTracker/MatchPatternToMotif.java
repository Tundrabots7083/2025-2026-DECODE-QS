package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ArtifactTracker;

import static org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor.GREEN;
import static org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor.PURPLE;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimeLight.DetectMotifPattern;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.ArtifactTracker;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor;

public class MatchPatternToMotif implements ActionFunction {

    Telemetry telemetry;

    SpindexerController spindexerController;
    ArtifactTracker artifactTracker;

    protected Status lastStatus = Status.FAILURE;
    private ArtifactColor[] currentPattern;
    private ArtifactColor[] targetPattern;
    private int numPurple = 0;
    private int numGreen = 0;
    private boolean isNotFull = true;
    private DetectMotifPattern.Pattern motifPattern;


    public MatchPatternToMotif(Telemetry telemetry, SpindexerController spindexerController, ArtifactTracker artifactTracker) {
        this.telemetry = telemetry;
        this.spindexerController = spindexerController;
        this.artifactTracker = artifactTracker;
    }

    public Status perform(BlackBoard blackBoard) {
        currentPattern = artifactTracker.getCurrentPattern();
        countArtifacts();

        if (isNotFull) {
            return Status.FAILURE;
        } else if (numPurple != 2) {
            blackBoard.setValue("isPatternConfigurable", false);
            return Status.SUCCESS;
        } else {
            blackBoard.setValue("isPatternConfigurable", true);
            motifPattern = (DetectMotifPattern.Pattern) blackBoard.getValue("Motif_Pattern");
        }

        if (motifPattern == null) {
            return Status.FAILURE;
        } else if (targetPattern == null) {
            createTargetPattern();
        }

        if (currentPattern == targetPattern) {
            blackBoard.setValue("spindexerTurnsToPattern", 0);
        } else if (currentPattern == targetPattern)

            return Status.SUCCESS;
    }

    private void countArtifacts() {
        for (ArtifactColor artifact : currentPattern) {
            if (artifact == ArtifactColor.PURPLE) {
                numPurple++;
            } else if (artifact == GREEN) {
                numGreen++;
            }
        }

        isNotFull = numGreen + numPurple != 3;
    }

    private void createTargetPattern() {
        switch (motifPattern) {
            case PPG:
                targetPattern = new ArtifactColor[]{PURPLE, PURPLE, GREEN};
                break;
            case GPP:
                targetPattern = new ArtifactColor[]{GREEN, PURPLE, PURPLE};
                break;
            case PGP:
                targetPattern = new ArtifactColor[]{PURPLE, GREEN, PURPLE};
                break;
        }
    }

    private ArtifactColor[] rotateArray(ArtifactColor[] arrayToRotate) {
        ArtifactColor[] tempArray = arrayToRotate;
        ArtifactColor[] arrayToReturn = arrayToRotate;

        for (int i = 0; i < 4; i++) {
            arrayToReturn[i % 3] = tempArray[i + 1];
        }

        return arrayToReturn;
    }
}