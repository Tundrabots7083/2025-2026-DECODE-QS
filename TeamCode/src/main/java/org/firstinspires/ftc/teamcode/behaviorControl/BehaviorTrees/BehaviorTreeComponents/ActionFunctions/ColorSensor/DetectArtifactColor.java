package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.RightIntakeColorSensorController;

public class DetectArtifactColor implements ActionFunction {

    Telemetry telemetry;

    RightIntakeColorSensorController rightColorSensorController;

    protected Status lastStatus = Status.FAILURE;
    private boolean isAutonomous = false;


    public DetectArtifactColor(Telemetry telemetry, RightIntakeColorSensorController rightColorSensorController) {
        this.telemetry = telemetry;
        this.rightColorSensorController = rightColorSensorController;
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS && isAutonomous) {
            return lastStatus;
        }

        isAutonomous = (boolean) blackBoard.getValue("isAutonomous");

        ArtifactColor rightColor = rightColorSensorController.getColor();
        blackBoard.setValue("ArtifactColor", rightColor);

        if (rightColor == ArtifactColor.NONE) {
            return Status.RUNNING;
        } else {
            lastStatus = Status.SUCCESS;
            return Status.SUCCESS;
        }
    }
}