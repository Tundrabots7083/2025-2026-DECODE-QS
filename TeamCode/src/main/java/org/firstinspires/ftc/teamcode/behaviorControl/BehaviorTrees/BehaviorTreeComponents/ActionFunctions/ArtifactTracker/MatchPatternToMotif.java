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

import java.util.Arrays;

public class MatchPatternToMotif implements ActionFunction {

    Telemetry telemetry;

    SpindexerController spindexerController;
    ArtifactTracker artifactTracker;

    protected Status lastStatus = Status.FAILURE;
    private ArtifactColor[] currentPattern;
    private ArtifactColor[] targetPattern;
    private int numPurple = 0;
    private int numGreen = 0;
    private boolean isFull = true;
    private DetectMotifPattern.Pattern motifPattern;


    public MatchPatternToMotif(Telemetry telemetry, SpindexerController spindexerController, ArtifactTracker artifactTracker) {
        this.telemetry = telemetry;
        this.spindexerController = spindexerController;
        this.artifactTracker = artifactTracker;
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
        currentPattern = artifactTracker.getCurrentPatternSnapshot();
        isFull = isFull();

        if (!isFull) {
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

        if (Arrays.equals(currentPattern, targetPattern)) {
            blackBoard.setValue("spindexerTurnsToPattern", 2);
        } else if (Arrays.equals(currentPattern, rotateArray(targetPattern))) {
            blackBoard.setValue("spindexerTurnsToPattern", 0);
        } else if (Arrays.equals(currentPattern, rotateArray(rotateArray(targetPattern)))) {
            blackBoard.setValue("spindexerTurnsToPattern", 1);
        }

            return Status.SUCCESS;
    }

    private boolean isFull() {
        ArtifactColor[] snapshot = artifactTracker.getCurrentPatternSnapshot();
        numPurple = 0;
        numGreen = 0;
        for (ArtifactColor artifact : snapshot) {
            if (artifact == ArtifactColor.PURPLE) {
                numPurple++;
            } else if (artifact == GREEN) {
                numGreen++;
            }
        }

        return (numGreen + numPurple) == 3;
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
        ArtifactColor[] arrayToReturn = new ArtifactColor[3];

        //shifts array to the right by 1 slot
        for (int i = 0; i <= 2; i++) {
            arrayToReturn[(i + 1) % 3] = arrayToRotate[i];
        }

        return arrayToReturn;
    }
}