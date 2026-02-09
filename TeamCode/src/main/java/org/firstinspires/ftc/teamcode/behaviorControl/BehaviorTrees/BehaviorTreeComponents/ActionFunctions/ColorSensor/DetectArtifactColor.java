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
    private boolean isAutonomous = false;


    public DetectArtifactColor(Telemetry telemetry, RightIntakeColorSensorController rightColorSensorController) {
        this.telemetry = telemetry;
        this.rightColorSensorController = rightColorSensorController;
        this.leftColorSensorController = LeftIntakeColorSensorController.getInstance();
    }

    public Status perform(BlackBoard blackBoard) {
        isAutonomous = (boolean) blackBoard.getValue("isAutonomous");

        if (lastStatus == Status.SUCCESS && isAutonomous) {
            return lastStatus;
        }


        ArtifactColor rightColor = rightColorSensorController.getColor();
        ArtifactColor leftColor = leftColorSensorController.getColor();

        if (rightColor != ArtifactColor.NONE) {
            blackBoard.setValue("ArtifactColor", rightColor);
        } else if (leftColor != ArtifactColor.NONE) {
            blackBoard.setValue("ArtifactColor", leftColor);
        } else {
            blackBoard.setValue("ArtifactColor", ArtifactColor.NONE);

        }

        if (rightColor == ArtifactColor.NONE && leftColor == ArtifactColor.NONE) {
            return Status.RUNNING;
        } else {
            lastStatus = Status.SUCCESS;
            return Status.SUCCESS;
        }
    }
}