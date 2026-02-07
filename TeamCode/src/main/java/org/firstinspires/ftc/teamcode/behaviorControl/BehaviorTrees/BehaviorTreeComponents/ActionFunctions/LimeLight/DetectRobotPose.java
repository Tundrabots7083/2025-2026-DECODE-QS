package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimeLight;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

//        double pinpointHeading = (double) blackBoard.getValue("CurrentPose");
//        double limelightHeading = (Math.toDegrees(pinpointHeading) + 90) % 360;
        Pose currentPose = limeLightController.getCurrentRobotPose();

        blackBoard.setValue("AprilTag_Pose", currentPose);


        status = Status.SUCCESS;

        return status;
    }
}