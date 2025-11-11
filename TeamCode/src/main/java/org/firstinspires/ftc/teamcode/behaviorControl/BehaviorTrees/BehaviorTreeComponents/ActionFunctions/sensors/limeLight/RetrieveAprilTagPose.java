package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.sensors.limeLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;

public class RetrieveAprilTagPose implements ActionFunction {
    Telemetry telemetry;
    LimeLightController limelightController;

    protected Status lastStatus = Status.FAILURE;


    public RetrieveAprilTagPose(Telemetry telemetry, LimeLightController limelightController) {
        this.telemetry = telemetry;
        this.limelightController = limelightController;
        this.init();
    }

    private void init() {
    limelightController.setPosePipeline();

    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        limelightController.update();

        Pose3D aprilTagPose = limelightController.getCurrentRobotPose();

        if(aprilTagPose != null) {
            blackBoard.setValue("AprilTagPose_X", aprilTagPose.getPosition().x);
            blackBoard.setValue("AprilTagPose_Y", aprilTagPose.getPosition().y);
            blackBoard.setValue("AprilTagPose_HEADING", aprilTagPose.getOrientation().getYaw(AngleUnit.DEGREES));
            status = Status.SUCCESS;
        } else {
            status = Status.FAILURE;
        }


        lastStatus = status;
        return status;
    }
}
