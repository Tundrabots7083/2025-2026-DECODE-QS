package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.LeftIntakeColorSensorController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.RightIntakeColorSensorController;

public class DetectArtifactColor implements ActionFunction {

    Telemetry telemetry;

    RightIntakeColorSensorController rightColorSensorController;
    LeftIntakeColorSensorController leftColorSensorController;

    protected Status lastStatus = Status.FAILURE;


    public DetectArtifactColor(Telemetry telemetry, RightIntakeColorSensorController rightColorSensorController, LeftIntakeColorSensorController leftColorSensorController) {
        this.telemetry = telemetry;
        this.rightColorSensorController = rightColorSensorController;
        this.leftColorSensorController = leftColorSensorController;
    }

    public Status perform(BlackBoard blackBoard) {

        ArtifactColor rightColor = rightColorSensorController.getColor();
        ArtifactColor leftColor = leftColorSensorController.getColor();

        if (rightColor == leftColor) {
            blackBoard.setValue("ArtifactColor", rightColorSensorController.getColor());
        } else if (rightColor == ArtifactColor.PURPLE || leftColor == ArtifactColor.PURPLE) {
            blackBoard.setValue("ArtifactColor", ArtifactColor.PURPLE);
        } else if (rightColor == ArtifactColor.GREEN || leftColor == ArtifactColor.GREEN) {
            blackBoard.setValue("ArtifactColor", ArtifactColor.GREEN);
        } else {
            blackBoard.setValue("ArtifactColor", ArtifactColor.NONE);
        }

        return Status.SUCCESS;
    }
}