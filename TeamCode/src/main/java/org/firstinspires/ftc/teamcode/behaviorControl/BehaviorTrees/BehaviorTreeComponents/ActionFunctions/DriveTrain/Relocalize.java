package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    ElapsedTime timer;

    public Relocalize(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;

        timer = new ElapsedTime();
    }


    public Status perform(BlackBoard blackBoard) {
        Status status;

        boolean isAutonomous = (boolean) blackBoard.getValue("isAutonomous");

        if (lastStatus == Status.SUCCESS && isAutonomous) {
            return lastStatus;
        }


        if (blackBoard.getValue("AprilTag_Pose") != null && timer.seconds() > 1) {
            Pose robotPose = (Pose) blackBoard.getValue("AprilTag_Pose");

            double x = robotPose.getX();
            double y = robotPose.getY();
            double heading = robotPose.getHeading();


            double pedroX = y + 72;
            double pedroY = -x + 72;
            double pedroHeading = (heading + 270) % 360;

            //Heading in radians

//            telemetry.addData("[RELOCALIZE] X", pedroX);
//            telemetry.addData("[RELOCALIZE] Y", pedroY);
//            telemetry.addData("[RELOCALIZE] Heading", pedroHeading);

            if (pedroX != 72.00 && pedroY != 72.00) {
                currentPose = new Pose(pedroX, pedroY, Math.toRadians(pedroHeading));

                driveTrainController.setPosition(currentPose);
                lastStatus = Status.SUCCESS;
            }

            timer.reset();
        }

        return Status.SUCCESS;
    }
}