package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.BLUE;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;


public class BLUEdriveToLEAVEfront implements ActionFunction {
    private DriveTrainController driveTrainController;
    private Telemetry telemetry;
    private Status lastStatus = Status.FAILURE;
    private final Pose shootPose = new Pose(50, 115, Math.toRadians(148));
    private PathChain shootPath;


    public BLUEdriveToLEAVEfront(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
    }

    @Override
    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        if (!driveTrainController.isBusy()) {

            shootPath = driveTrainController.pathBuilder()
                    .addPath(new BezierLine(driveTrainController::getPosition, shootPose))
                    .setLinearHeadingInterpolation(driveTrainController.getPosition().getHeading(), shootPose.getHeading())
                    .build();

            driveTrainController.followPath(shootPath, true);
            lastStatus = Status.SUCCESS;
            return Status.SUCCESS;
        }

        lastStatus = Status.RUNNING;
        return Status.RUNNING;
    }
}