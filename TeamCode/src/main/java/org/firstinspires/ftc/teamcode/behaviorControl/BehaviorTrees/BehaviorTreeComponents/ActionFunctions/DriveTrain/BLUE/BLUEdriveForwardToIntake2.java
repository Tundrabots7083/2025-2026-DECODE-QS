package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.BLUE;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;


public class BLUEdriveForwardToIntake2 implements ActionFunction {
    private DriveTrainController driveTrainController;
    private Telemetry telemetry;
    private Status lastStatus = Status.FAILURE;
    private double maxPower = 0.35;
    private final Pose startPose = new Pose(42, 60, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose intakePose = new Pose(24, 60, Math.toRadians(180));
    private PathChain intakePath;


    public BLUEdriveForwardToIntake2(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;

        intakePath = driveTrainController.pathBuilder()
                .addPath(new BezierLine(startPose, intakePose))
                .build();
    }

    @Override
    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        if (!driveTrainController.isBusy()) {
            driveTrainController.followPath(intakePath, maxPower, true);
            lastStatus = Status.SUCCESS;
            return Status.SUCCESS;
        }

        lastStatus = Status.RUNNING;
        return Status.RUNNING;
    }
}