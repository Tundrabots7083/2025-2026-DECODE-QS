package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;


public class WaitForDrivetrainToArrive implements ActionFunction {
    private DriveTrainController driveTrainController;
    private Telemetry telemetry;
    Status lastStatus = Status.FAILURE;

    public WaitForDrivetrainToArrive(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
    }

    @Override
    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
        if (!driveTrainController.isBusy()) {
            telemetry.addData("SFDJLKJDFKJSLKJFLSKDFKLSDKLJFKLJSDKLF", "dksajkdlfjkkljdfsl");
            lastStatus = Status.SUCCESS;
        } else {
            lastStatus = Status.RUNNING;
        }
        return lastStatus;
    }
}