package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.ArtifactTracker;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;

enum PrepState {
    IDLE,
    SWITCH_COORDINATES,
    SPIN_UP,
    DEPLOY_RAMP,
    READY
}

public class ShootPrep implements ActionFunction {
    protected Status lastStatus = Status.FAILURE;
    Telemetry telemetry;
    ShooterController shooterController;
    RampController rampController;
    SpindexerController spindexerController;
    ArtifactTracker artifactTracker;
    IntakeController intakeController;
    PrepState state = PrepState.IDLE;
    boolean wasLeftTriggerTripped = false;
    boolean wasRightTriggerTripped = false;
    boolean isShootingThree = false;
    int timesSpun = 0;

    public ShootPrep(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.shooterController = ShooterController.getInstance();
        this.rampController = RampController.getInstance();
        this.spindexerController = SpindexerController.getInstance();
        this.artifactTracker = ArtifactTracker.getInstance();
        this.intakeController = IntakeController.getInstance();
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
                    state = PrepState.SWITCH_COORDINATES;
                    isShootingThree = false;
                    break;
                } else if (wasRightTriggerTripped) {
                    state = PrepState.SWITCH_COORDINATES;
                    isShootingThree = true;
                    break;
                }
                return Status.FAILURE;
            case SWITCH_COORDINATES:
                spindexerController.setDegreeOffset(0.0);

                double currentPosition = spindexerController.getPosition() % 360;

                if (currentPosition > 240) {
                    spindexerController.moveToPosition(spindexerController.getTargetPosition());
                } else if (currentPosition > 120) {
                    spindexerController.moveToPosition(spindexerController.getTargetPosition() + 240);
                } else {
                    spindexerController.moveToPosition(spindexerController.getTargetPosition() + 120);
                }
                state = PrepState.IDLE;
                break;
            case DEPLOY_RAMP:
                if (spindexerController.isOnTarget()) {
                    rampController.deploy();
                    if (rampController.isDeployed()) {
                        state = PrepState.SPIN_UP;
                    }
                }
                break;
            case SPIN_UP:
                shooterController.spinToTargetVelocity(3500);
                state = PrepState.READY;
                break;
        }

        return Status.SUCCESS;

    }
}