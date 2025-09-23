package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.FrontZone;

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
    private final Pose scorePose = new Pose(28, 63, Math.toRadians(0));
    private final Pose ctrlPt1 = new Pose(30.3, 42.6, Math.toRadians(0));
    private final Pose ctrlPt2 = new Pose(64.5, 37.7, Math.toRadians(0));
    private final Pose moveSample2Pose = new Pose(61, 24, Math.toRadians(0));
    private final Pose moveSample2StagingPose = new Pose(17, 16.5, Math.toRadians(0));
    private final Pose ctrlPt3 = new Pose(31.7, 24.3, Math.toRadians(0));


    PathChain moveSample2;
    boolean started = false;


    public MoveToShootingPosition(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
        this.init();
    }

    private void init() {
        moveSample2 = driveTrainController.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                ctrlPt1,
                                ctrlPt2,
                                moveSample2Pose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                moveSample2Pose,
                                ctrlPt3,
                                moveSample2StagingPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
        /// TODO: put code here to move the robot from current position to push sample2 in staging area
        if (!started) {
            driveTrainController.followPath(moveSample2, true);
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

