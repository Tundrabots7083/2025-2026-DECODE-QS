package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.gamepad.GamepadSnapshot;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;

public class TeleOpDrive implements ActionFunction {
    Telemetry telemetry;
    DriveTrainController driveTrainController;
    GamepadSnapshot gamepad1;
    boolean hasRun = false;

    protected Status lastStatus = Status.FAILURE;

    private Pose currentPose;

    public TeleOpDrive(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
    }


    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (!hasRun) {
            driveTrainController.startTeleOpDrive();
            hasRun = true;
        }

        if (blackBoard.getValue("gamepad1Snapshot") != null) {

            gamepad1 = (GamepadSnapshot) blackBoard.getValue("gamepad1Snapshot");

            driveTrainController.setTeleOpDrive(
                    -gamepad1.leftStickY,
                    -gamepad1.leftStickX,
                    -gamepad1.rightStickX,
                    true // Robot Centric
            );
            status = Status.SUCCESS;
        } else {
            status = Status.FAILURE;
        }


        lastStatus = status;
        return status;
    }

}