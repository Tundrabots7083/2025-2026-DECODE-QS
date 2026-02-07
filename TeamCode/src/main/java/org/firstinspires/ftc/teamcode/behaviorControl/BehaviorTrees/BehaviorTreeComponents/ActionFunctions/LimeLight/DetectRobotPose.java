package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimeLight;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;

public class DetectRobotPose implements ActionFunction {

    protected Status lastStatus = Status.FAILURE;
    Telemetry telemetry;
    LimeLightController limeLightController;


    public DetectRobotPose(Telemetry telemetry, LimeLightController limeLightController) {
        this.telemetry = telemetry;
        this.limeLightController = limeLightController;
    }

    public Status perform(BlackBoard blackBoard) {
        Status status;
        boolean isAutonomous = (boolean) blackBoard.getValue("isAutonomous");

        if (lastStatus == Status.SUCCESS && isAutonomous) {
            return lastStatus;
        }

//        double pinpointHeading = (double) blackBoard.getValue("CurrentPose");
//        double limelightHeading = (Math.toDegrees(pinpointHeading) + 90) % 360;
        Pose currentPose = limeLightController.getCurrentRobotPose();

        if (currentPose != null) {
            lastStatus = Status.SUCCESS;
        }

        blackBoard.setValue("AprilTag_Pose", currentPose);


        status = Status.SUCCESS;

        return status;
    }
}