package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.IntakeColorSensorController;

public class DetectArtifactColor implements ActionFunction {

    Telemetry telemetry;

    IntakeColorSensorController intakeColorSensorController;

    protected Status lastStatus = Status.FAILURE;


    public DetectArtifactColor(Telemetry telemetry, IntakeColorSensorController intakeColorSensorController) {
        this.telemetry = telemetry;
        this.intakeColorSensorController = intakeColorSensorController;
    }

    public Status perform(BlackBoard blackBoard) {

        blackBoard.setValue("ArtifactColor", intakeColorSensorController.getColor());

        return Status.SUCCESS;
    }
}