package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ArtifactTracker;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.ArtifactTracker;

public class ClearArtifactTracker implements ActionFunction {

    Telemetry telemetry;

    ArtifactTracker artifactTracker;

    protected Status lastStatus = Status.FAILURE;


    public ClearArtifactTracker(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.artifactTracker = ArtifactTracker.getInstance();
    }

    public Status perform(BlackBoard blackBoard) {

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        artifactTracker.clearPattern();

        lastStatus = Status.SUCCESS;
        return Status.SUCCESS;
    }
}