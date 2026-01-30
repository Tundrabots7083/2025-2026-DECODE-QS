package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;

public class ShootAction implements ActionFunction {
    Telemetry telemetry;
    ShooterController shooterController;
    RampController rampController;
    SpindexerController spindexerController;

    protected Status lastStatus = Status.FAILURE;
    ShootState state = ShootState.IDLE;
    boolean wasTriggerTripped = false;

    public ShootAction(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.shooterController = ShooterController.getInstance();
        this.rampController = RampController.getInstance();
        this.spindexerController = SpindexerController.getInstance();
    }


    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (blackBoard.getValue("gamepad1Delta") != null) {
            GamepadDelta gamepad1Delta = (GamepadDelta) blackBoard.getValue("gamepad1Delta");
            wasTriggerTripped = gamepad1Delta.leftTriggerPulling;
        } else {
//            return Status.FAILURE;
        }

        switch (state) {
            case IDLE:
                if (wasTriggerTripped) {
                    state = ShootState.DEPLOY_RAMP;
                    return Status.SUCCESS;
                } else {
//                    return Status.FAILURE;
                }
                break;
            case DEPLOY_RAMP:
                rampController.deploy();
                if (rampController.isDeployed()) {
                    state = ShootState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                shooterController.spinToTargetVelocity(100);
                if (shooterController.isOnTarget()) {
                    state = ShootState.FEED;
                }
                break;
            case FEED:
                double currentTarget = spindexerController.getTargetPosition();
                double targetPosition = currentTarget + 120;
                spindexerController.moveToPosition(targetPosition);
                if (spindexerController.isOnTarget()) {
                    state = ShootState.RETRACT;
                }
                break;
            case RETRACT:
                rampController.store();
                if (!rampController.isDeployed()) {
                    state = ShootState.IDLE;
                }
                break;
        }

        return Status.SUCCESS;

    }
}

enum ShootState {
    IDLE,
    SPIN_UP,
    DEPLOY_RAMP,
    FEED,
    RETRACT
}