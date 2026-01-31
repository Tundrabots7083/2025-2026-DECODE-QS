package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;

enum ShootState {
    IDLE,
    SPIN_UP,
    DEPLOY_RAMP,
    FEED,
    RETRACT
}

public class ShootAction implements ActionFunction {
    protected Status lastStatus = Status.FAILURE;
    Telemetry telemetry;
    ShooterController shooterController;
    RampController rampController;
    SpindexerController spindexerController;
    ShootState state = ShootState.IDLE;
    boolean wasLeftTriggerTripped = false;
    boolean wasRightTriggerTripped = false;
    boolean isShootingThree = false;
    int timesSpun = 0;

    public ShootAction(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.shooterController = ShooterController.getInstance();
        this.rampController = RampController.getInstance();
        this.spindexerController = SpindexerController.getInstance();
    }


    public Status perform(BlackBoard blackBoard) {

        if (blackBoard.getValue("gamepad1Delta") != null) {
            GamepadDelta gamepad1Delta = (GamepadDelta) blackBoard.getValue("gamepad1Delta");
            wasLeftTriggerTripped = gamepad1Delta.leftTriggerPulling;
            wasRightTriggerTripped = gamepad1Delta.rightTriggerPulling;
        } else {
            return Status.SUCCESS;
        }

        switch (state) {
            case IDLE:
                timesSpun = 0;
                if (wasLeftTriggerTripped) {
                    state = ShootState.DEPLOY_RAMP;
                    isShootingThree = false;
                    break;
                } else if (wasRightTriggerTripped) {
                    state = ShootState.DEPLOY_RAMP;
                    isShootingThree = true;
                    break;
                }
                break;
            case DEPLOY_RAMP:
                rampController.deploy();
                if (rampController.isDeployed()) {
                    state = ShootState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                shooterController.spinToTargetVelocity(0);
                state = ShootState.FEED;
                break;
            case FEED:
                if (shooterController.isOnTarget()) {
                    double currentTarget = spindexerController.getTargetPosition();
                    double targetPosition = currentTarget + 120;
                    spindexerController.moveToPosition(targetPosition);
                    timesSpun++;
                    state = ShootState.RETRACT;
                }
                break;
            case RETRACT:
                if (spindexerController.isOnTarget()) {
                    if (!isShootingThree || timesSpun == 3) {
                        shooterController.spinToTargetVelocity(0.0);
                        rampController.store();
                    } else {
                        state = ShootState.SPIN_UP;
                        break;
                    }
                }
                if (!rampController.isDeployed()) {
                    state = ShootState.IDLE;
                }
                break;
        }

        return Status.SUCCESS;

    }
}