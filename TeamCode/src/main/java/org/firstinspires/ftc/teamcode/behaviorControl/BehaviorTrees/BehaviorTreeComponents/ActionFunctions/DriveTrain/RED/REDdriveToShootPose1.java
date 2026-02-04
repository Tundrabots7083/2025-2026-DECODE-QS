package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.RED;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;


public class REDdriveToShootPose1 implements ActionFunction {
    private DriveTrainController driveTrainController;
    private Telemetry telemetry;
    private Status lastStatus = Status.FAILURE;
    private final Pose intakePose = new Pose(87, 17, Math.toRadians(90));
    private PathChain intakePath;


    public REDdriveToShootPose1(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
    }

    @Override
    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        if (!driveTrainController.isBusy()) {

            intakePath = driveTrainController.pathBuilder()
                    .addPath(new BezierLine(driveTrainController::getPosition, intakePose))
                    .setLinearHeadingInterpolation(driveTrainController.getPosition().getHeading(), intakePose.getHeading())
                    .build();

            driveTrainController.followPath(intakePath, true);
            lastStatus = Status.SUCCESS;
            return Status.SUCCESS;
        }

        lastStatus = Status.RUNNING;
        return Status.RUNNING;
    }
}