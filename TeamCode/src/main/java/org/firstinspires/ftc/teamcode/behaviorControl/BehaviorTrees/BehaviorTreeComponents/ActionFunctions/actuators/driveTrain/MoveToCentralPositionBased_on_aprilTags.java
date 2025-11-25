package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.actuators.driveTrain;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;

public class MoveToCentralPositionBased_on_aprilTags implements ActionFunction {
    Telemetry telemetry;
    DriveTrainController driveTrainController;

    protected Status lastStatus = Status.FAILURE;

    private final Pose centralPose = new Pose(72, 72);

    private Supplier<PathChain> moveToCentralPose;
    boolean started = false;


    public MoveToCentralPositionBased_on_aprilTags(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
        this.init();
    }

    private void init() {
        moveToCentralPose = () -> driveTrainController.pathBuilder()
                .addPath(
                        new Path(
                        new BezierLine(
                                driveTrainController::getPosition,
                                centralPose
                        )
                        )
                )
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(driveTrainController::getHeading, Math.toRadians(45), 0.99))
                .build();
    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        driveTrainController.update();

        if (!started) {
            driveTrainController.followPath(moveToCentralPose.get(), true);
            started = true;
            status = Status.RUNNING;
        } else {
            if (driveTrainController.isBusy()) {
                status = Status.RUNNING;
            } else {
                status = Status.SUCCESS;
            }
        }


        lastStatus = status;
        return status;
    }

}

