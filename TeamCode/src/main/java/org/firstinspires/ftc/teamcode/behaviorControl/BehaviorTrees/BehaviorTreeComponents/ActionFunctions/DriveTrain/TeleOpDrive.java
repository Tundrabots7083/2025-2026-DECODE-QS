package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.gamepad.GamepadSnapshot;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;

public class TeleOpDrive implements ActionFunction {
    Telemetry telemetry;
    DriveTrainController driveTrainController;
    GamepadSnapshot gamepad1;
    GamepadDelta gamepad2Delta;
    boolean hasRun = false;

    double slowMultiplier = 0.5;


    protected Status lastStatus = Status.FAILURE;

    private boolean slowMode = false;

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

        if (blackBoard.getValue("gamepad2Delta") != null) {
            gamepad2Delta = (GamepadDelta) blackBoard.getValue("gamepad2Delta");
            if (gamepad2Delta.rightBumperPressed) {
                slowMode = !slowMode;
            }
        }


        if (blackBoard.getValue("gamepad1Snapshot") != null) {

            gamepad1 = (GamepadSnapshot) blackBoard.getValue("gamepad1Snapshot");

            if (!slowMode) {
                driveTrainController.setTeleOpDrive(
                        -gamepad1.leftStickY,
                        -gamepad1.leftStickX,
                        -gamepad1.rightStickX,
                        true // Robot Centric
                );
            } else {
                driveTrainController.setTeleOpDrive(
                        -gamepad1.leftStickY * slowMultiplier,
                        -gamepad1.leftStickX * slowMultiplier,
                        -gamepad1.rightStickX * slowMultiplier,
                        true // Robot Centric
                );
            }
            status = Status.SUCCESS;
        } else {
            status = Status.FAILURE;
        }


        lastStatus = status;
        return status;
    }

}