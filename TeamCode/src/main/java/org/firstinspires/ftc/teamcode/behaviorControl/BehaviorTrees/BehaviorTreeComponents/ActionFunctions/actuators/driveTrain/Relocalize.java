package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.actuators.driveTrain;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;

public class Relocalize implements ActionFunction {
    Telemetry telemetry;
    DriveTrainController driveTrainController;

    protected Status lastStatus = Status.FAILURE;

    private Pose currentPose;

    public Relocalize(Telemetry telemetry, DriveTrainController driveTrainController) {
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
        } else if (blackBoard.getValue("AprilTagPose_X") != null) {

            double x = (double) blackBoard.getValue("AprilTagPose_X");
            double y = (double) blackBoard.getValue("AprilTagPose_Y");
            double heading = Math.toRadians(((double) blackBoard.getValue("AprilTagPose_HEADING") + 270) % 360);


            double pedroX = y + 72;
            double pedroY = -x + 72;
            double pedroHeading = (heading + 270) % 360;

            currentPose = new Pose(pedroX, pedroY, pedroHeading);

            driveTrainController.breakFollowing();
            driveTrainController.setPosition(currentPose);
            telemetry.addLine("SLKFJLKJSFLKJDSLFJSKFJLSDKFKSLDKJFLSKJDFLSKJLDFKJSLDJFLSKJDFLKJSDLFKJDSLFKJ");
            status = Status.SUCCESS;
        } else {
            status = Status.FAILURE;
        }

        lastStatus = status;
        return status;
    }

}

