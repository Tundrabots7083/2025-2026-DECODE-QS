package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;

public class CalculateDistanceToGoal implements ActionFunction {
    Telemetry telemetry;
    DriveTrainController driveTrainController;

    protected Status lastStatus = Status.FAILURE;

    private Pose currentPose;
    private Pose goalPose = new Pose(100, 100, 0);

    public CalculateDistanceToGoal(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
    }


    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }


        if (driveTrainController.isBusy()) {
            status = Status.RUNNING;
        } else if (driveTrainController.isRobotStuck()) {
            status = Status.FAILURE;
        } else {
            currentPose = driveTrainController.getPosition();
            blackBoard.setValue("DistanceToGoal", getDistance());

            status = Status.SUCCESS;
        }

        lastStatus = status;
        return status;
    }

    private double getDistance() {
        double distanceX = goalPose.getX() - currentPose.getX();
        double distanceY = goalPose.getY() - currentPose.getY();

        return Math.sqrt(
                Math.pow(distanceX, 2) + Math.pow(distanceY, 2)
        );

    }

}