package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimeLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;

import java.util.Set;

public class DetectMotifPattern implements ActionFunction {

    Telemetry telemetry;

    LimeLightController limeLightController;

    protected Status lastStatus = Status.FAILURE;


    public DetectMotifPattern(Telemetry telemetry, LimeLightController limeLightController) {
        this.telemetry = telemetry;
        this.limeLightController = limeLightController;
    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        Set<Integer> ids = limeLightController.getPresentFiducialIds();


        if (ids.contains(21)) {
            blackBoard.setValue("Motif_Pattern", Pattern.GPP);
        } else if (ids.contains(22)) {
            blackBoard.setValue("Motif_Pattern", Pattern.PGP);
        } else if (ids.contains(23)) {
            blackBoard.setValue("Motif_Pattern", Pattern.PPG);
        } else {
            status = Status.FAILURE;
            return status;
        }

        status = Status.SUCCESS;

        return status;
    }

    enum Pattern {
        PPG,
        PGP,
        GPP
    }
}