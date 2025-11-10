package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.actuators.driveTrain;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;

public class MoveToShootingPosition implements ActionFunction {
    Telemetry telemetry;
    DriveTrainController driveTrainController;

    protected Status lastStatus = Status.FAILURE;

    /// TODO: set the right poses
    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(0));
    private final Pose ctrlPt1 = new Pose(67.309, 18.293, Math.toRadians(0));
    private final Pose ctrlPt2 = new Pose(77.394, 34.007, Math.toRadians(0));
    private final Pose shootingPose = new Pose(54.410, 36.586, Math.toRadians(0));

    PathChain moveToShootingPose;
    boolean started = false;


    public MoveToShootingPosition(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
        this.init();
    }

    private void init() {
        moveToShootingPose = driveTrainController.pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                ctrlPt1,
                                ctrlPt2,
                                shootingPose
                        )
                )
                .build();

    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }


        if (!started) {
            driveTrainController.followPath(moveToShootingPose, true);
            started = true;
            status = Status.RUNNING;
        } else {
            if (driveTrainController.isBusy()) {
                status = Status.RUNNING;
            } else {
                if (driveTrainController.isRobotStuck()) {
                    status = Status.FAILURE;
                } else {
                    status = Status.SUCCESS;
                }
            }
        }

        driveTrainController.update();
        lastStatus = status;
        return status;
    }

}

