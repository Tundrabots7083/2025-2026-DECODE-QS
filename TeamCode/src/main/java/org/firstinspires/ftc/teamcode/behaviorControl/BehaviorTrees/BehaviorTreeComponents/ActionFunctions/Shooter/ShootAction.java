package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.ArtifactTracker;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.spindexerLimitSwitch.SpindexerLimitSwitchController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor;

enum ShootState {
    IDLE,
    SWITCH_COORDINATES,
    RUN_TO_ZERO,
    DEPLOY_RAMP,
    SPIN_UP,
    WAIT_FOR_TRIGGER,
    FEED,
    RETRACT,
    CALIBRATE_SPINDEXER
}

@Configurable
public class ShootAction implements ActionFunction {
    protected Status lastStatus = Status.FAILURE;
    Telemetry telemetry;
    ShooterController shooterController;
    RampController rampController;
    SpindexerController spindexerController;
    ArtifactTracker artifactTracker;
    IntakeController intakeController;
    SpindexerLimitSwitchController limitSwitchController;
    DriveTrainController driveTrainController;
    ShootState state = ShootState.IDLE;
    boolean wasLeftTriggerTripped = false;
    boolean wasRightTriggerTripped = false;
    boolean wasLeftBumperPressed = false;
    boolean wasRightBumperPressed = false;
    boolean isShootingThree = false;
    boolean cancelConsumed = false;
    int timesSpun = 0;
//    Pose robotPose;

    public static double shooterRPM = 3500;
    private double spindexerPower = 0.3;

    public ShootAction(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.shooterController = ShooterController.getInstance();
        this.rampController = RampController.getInstance();
        this.spindexerController = SpindexerController.getInstance();
        this.artifactTracker = ArtifactTracker.getInstance();
        this.intakeController = IntakeController.getInstance();
        this.limitSwitchController = SpindexerLimitSwitchController.getInstance();
//        this.driveTrainController = DriveTrainController.getInstance();
    }


    public Status perform(BlackBoard blackBoard) {

        if (blackBoard.getValue("gamepad1Delta") != null) {
            GamepadDelta gamepad1Delta = (GamepadDelta) blackBoard.getValue("gamepad1Delta");
            wasLeftTriggerTripped = gamepad1Delta.leftTriggerPulling;
            wasRightTriggerTripped = gamepad1Delta.rightTriggerPulling;
            wasLeftBumperPressed = gamepad1Delta.leftBumperPressed;
            wasRightBumperPressed = gamepad1Delta.rightBumperPressed;
        }

        if (blackBoard.getValue("TargetShooterRPM") != null) {
            shooterRPM = (double) blackBoard.getValue("TargetShooterRPM");
            spindexerPower = (double) blackBoard.getValue("SpindexerPower");

//            telemetry.addData("[SHOOT ACTION] shooterRPM",shooterRPM);
        }

//        if(blackBoard.getValue("CurrentPose") != null) {
//            robotPose = (Pose) blackBoard.getValue("CurrentPose");
//        }

//        telemetry.addData("[SHOOTACTION] Front Speed", shooterController.getFrontCurrentVelocity());
//        telemetry.addData("[SHOOTACTION] Reare Speed", shooterController.getRearCurrentVelocity());

        switch (state) {
            case IDLE:
                timesSpun = 0;
                isShootingThree = false;
                shooterController.spinToTargetVelocity(0.0);
                if (wasLeftBumperPressed || wasRightBumperPressed) {
                    state = ShootState.SWITCH_COORDINATES;
                    intakeController.spinToTargetVelocity(200);
                    break;
                } else {
                    cancelConsumed = false;
                }
                return Status.SUCCESS;
            case SWITCH_COORDINATES:
                spindexerController.setDegreeOffset(-50.0);
                spindexerController.moveToPosition(spindexerController.getTargetPosition() - 50);
                state = ShootState.RUN_TO_ZERO;
                break;
            case RUN_TO_ZERO:
                if (spindexerController.isOnTarget()) {
                    spindexerController.moveToPosition(spindexerController.getTargetPosition() + 50);
                    state = ShootState.DEPLOY_RAMP;
                }
                break;
            case DEPLOY_RAMP:
                if (spindexerController.isOnTarget()) {
                    rampController.deploy();

                    if (rampController.isDeployed()) {
                        state = ShootState.SPIN_UP;
                    }
                }

                break;
            case SPIN_UP:
                shooterController.spinToTargetVelocity(shooterRPM);
                if (isShootingThree) {
                    state = ShootState.FEED;
                    break;
                }
                state = ShootState.WAIT_FOR_TRIGGER;
                break;
            case WAIT_FOR_TRIGGER:
                if (wasRightBumperPressed || wasLeftBumperPressed) {

                    shooterController.spinToTargetVelocity(0.0);
                    intakeController.spinToTargetVelocity(0.0);
                    rampController.store();
                    spindexerController.setDegreeOffset(-10);
                    if (!rampController.isDeployed()) {
                        state = ShootState.IDLE;
                    }

                }
                shooterController.spinToTargetVelocity(shooterRPM);
                if (wasLeftTriggerTripped) {
                    isShootingThree = false;
                    state = ShootState.FEED;
                    break;
                } else if (wasRightTriggerTripped) {
                    isShootingThree = true;
                    state = ShootState.FEED;
                    break;
                }
                break;
            case FEED:
                if (shooterController.isOnTarget()) {
                    intakeController.spinToTargetVelocity(200);
                    int shootSlot = (spindexerController.getSlotPosition() + 1) % 3; // gets the slot currently under the shooter
                    artifactTracker.setArtifact(shootSlot, ArtifactColor.NONE);
                    double currentTarget = spindexerController.getTargetPosition();
                    double targetPosition;
                    if (isShootingThree) {
                        artifactTracker.clearPattern();
                        targetPosition = currentTarget + 360;
                    } else {
                        targetPosition = currentTarget + 120;
                    }
                    spindexerController.testSpindexer(0.3);
                    spindexerController.moveToPosition(targetPosition);
                    state = ShootState.RETRACT;
                }
                break;
            case RETRACT:
                if (spindexerController.isPastTarget()) {
                        shooterController.spinToTargetVelocity(0.0);
                        rampController.store();
                }
                if (!rampController.isDeployed()) {
                    state = ShootState.CALIBRATE_SPINDEXER;
                }
                break;
            case CALIBRATE_SPINDEXER:
                boolean switchState = limitSwitchController.getState();
                if (!switchState) {
                    spindexerController.spinSlowly();
                } else {
                    spindexerController.stop();
                    spindexerController.hardwareReset();
                    spindexerController.setDegreeOffset(-10);
                    spindexerController.moveToPosition(120);
                    intakeController.stop();
                    state = ShootState.IDLE;
                }
                break;
        }

        return Status.SUCCESS;

    }
}