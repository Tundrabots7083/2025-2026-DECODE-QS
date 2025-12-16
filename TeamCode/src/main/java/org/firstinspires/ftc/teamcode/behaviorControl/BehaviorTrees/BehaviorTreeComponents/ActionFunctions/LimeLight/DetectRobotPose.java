package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimeLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;

public class DetectRobotPose implements ActionFunction {

    Telemetry telemetry;

    LimeLightController limeLightController;

    protected Status lastStatus = Status.FAILURE;


    public DetectRobotPose(Telemetry telemetry, LimeLightController limeLightController) {
        this.telemetry = telemetry;
        this.limeLightController = limeLightController;
    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        Pose3D currentPose = limeLightController.getCurrentRobotPose();

        if (currentPose != null) {
            blackBoard.setValue("AprilTag_Pose", currentPose);
        } else {
            return Status.FAILURE;
        }

        status = Status.SUCCESS;

        return status;
    }
}