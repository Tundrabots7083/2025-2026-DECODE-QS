package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.sensors.limeLight;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;

import java.util.Collections;
import java.util.List;

public class DetectMotif implements ActionFunction {
    Telemetry telemetry;
    LimeLightController limelightController;

    protected Status lastStatus = Status.FAILURE;

    boolean started = false;
    private List<LLResultTypes.FiducialResult> fiducialResults = Collections.emptyList();


    public DetectMotif(Telemetry telemetry, LimeLightController limelightController) {
        this.telemetry = telemetry;
        this.limelightController = limelightController;
        this.init();
    }

    private void init() {
    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        limelightController.setMotifPipeline();


        fiducialResults = limelightController.getFiducialResults();
        status = Status.RUNNING;

        if(!fiducialResults.isEmpty()) {
            blackBoard.setValue("FiducialResults", fiducialResults);
            status = Status.SUCCESS;
        }


        limelightController.update();
        lastStatus = status;
        return status;
    }
}
