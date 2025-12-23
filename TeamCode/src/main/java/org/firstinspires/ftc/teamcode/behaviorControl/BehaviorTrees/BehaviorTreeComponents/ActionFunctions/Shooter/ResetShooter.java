package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;

public class ResetShooter implements ActionFunction {
    Telemetry telemetry;
    ShooterController shooterController;
    protected Status lastStatus = Status.FAILURE;

    boolean started = false;

    public ResetShooter(Telemetry telemetry, ShooterController shooterController) {
        this.telemetry = telemetry;
        this.shooterController = shooterController;
        this.init();
    }

    private void init() {
    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
        if (!started) {
            shooterController.reset();
            started = true;
            status = Status.RUNNING;
        } else {
            if (!shooterController.isOnTarget()) {
                status = Status.RUNNING;
            } else {
                status = Status.SUCCESS;
            }
        }

        return status;
    }

}
