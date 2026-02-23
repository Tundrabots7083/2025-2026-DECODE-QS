package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake;

import static org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor.GREEN;
import static org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor.PURPLE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.ArtifactTracker;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.LeftIntakeColorSensorController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.RightIntakeColorSensorController;


enum IntakeState {
    IDLE,
    INTAKE_ARTIFACTS,
    STORE_RAMP,
    CHECK_SLOT,
    SPIN_SPINDEXER,
    REJECT_EXTRA_ARTIFACTS
}

public class IntakeAction implements ActionFunction {
    protected Status lastStatus = Status.FAILURE;
    Telemetry telemetry;
    IntakeController intakeController;
    RampController rampController;
    SpindexerController spindexerController;
    RightIntakeColorSensorController rightColorSensorController;
    LeftIntakeColorSensorController leftColorSensorController;
    ArtifactTracker artifactTracker;
    IntakeState state = IntakeState.IDLE;
    boolean wasAPressed = false;
    boolean wasDPadDownPressed = false;
    double REJECT_VELOCITY = 200;
    double INTAKE_VELOCITY = 300;
    int lastSlot = 0;
    boolean wasRightReadLast = false;
    LinearOpMode opMode;

    public IntakeAction(Telemetry telemetry, LinearOpMode opMode) {
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.intakeController = IntakeController.getInstance();
        this.rampController = RampController.getInstance();
        this.spindexerController = SpindexerController.getInstance();
        this.rightColorSensorController = RightIntakeColorSensorController.getInstance();
        this.leftColorSensorController = LeftIntakeColorSensorController.getInstance();
        this.artifactTracker = ArtifactTracker.getInstance();
    }


    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (blackBoard.getValue("gamepad1Delta") != null) {
            GamepadDelta gamepad1Delta = (GamepadDelta) blackBoard.getValue("gamepad1Delta");
            wasAPressed = gamepad1Delta.aPressed;
        } else {
//            return Status.FAILURE;
        }

        if (blackBoard.getValue("gamepad2Delta") != null) {
            GamepadDelta gamepad2Delta = (GamepadDelta) blackBoard.getValue("gamepad2Delta");
            wasDPadDownPressed = gamepad2Delta.rightTriggerPulling || gamepad2Delta.leftTriggerPulling;
        } else {
//            return Status.FAILURE;
        }


        telemetry.addData("[INTAKE ACTION] Atrifact Tracker 1", artifactTracker.getArtifact(0));
        telemetry.addData("[INTAKE ACTION] Atrifact Tracker 2", artifactTracker.getArtifact(1));
        telemetry.addData("[INTAKE ACTION] Atrifact Tracker 3", artifactTracker.getArtifact(2));
//        telemetry.addData("[INTAKE ACTION] State", state);

        switch (state) {
            case IDLE:
                if (wasAPressed) {
                    state = IntakeState.STORE_RAMP;
                    return Status.SUCCESS;
                } else {
                    return Status.FAILURE;
                }
            case STORE_RAMP:
                if (wasAPressed) {
                    state = IntakeState.IDLE;
                    intakeController.stop();
                    return Status.SUCCESS;
                }
                if (rampController.isDeployed()) {
                    rampController.store();
                } else {
                    state = IntakeState.INTAKE_ARTIFACTS;
                }
                break;
            case INTAKE_ARTIFACTS:
                if (wasAPressed) {
                    intakeController.stop();
                    state = IntakeState.IDLE;
                    return Status.SUCCESS;
                }
                intakeController.spinToTargetVelocity(INTAKE_VELOCITY);
                if (spindexerController.isOnTarget()) {
                    state = IntakeState.CHECK_SLOT;
                }
                break;
            case CHECK_SLOT:
                if (wasAPressed) {
                    intakeController.stop();
                    state = IntakeState.IDLE;
                    return Status.SUCCESS;
                }

                //Ensure intake is still spinning if interrupted
                intakeController.spinToTargetVelocity(INTAKE_VELOCITY);

                ArtifactColor rightColor = rightColorSensorController.getColor();
                ArtifactColor leftColor = leftColorSensorController.getColor();
                wasRightReadLast = !wasRightReadLast;

                int slotPosition = spindexerController.getSlotPosition();
                if (rightColor != ArtifactColor.NONE || wasDPadDownPressed) {
                    intakeController.spinToTargetVelocity(REJECT_VELOCITY);
//                    artifactTracker.setArtifact(slotPosition, rightColor);
                    artifactTracker.setArtifact(slotPosition, PURPLE);

                    if (isSpindexerFull()) {
                        opMode.gamepad1.rumble(200);
                        state = IntakeState.IDLE;
                    } else {
                        state = IntakeState.SPIN_SPINDEXER;
                    }
                } else if (leftColor != ArtifactColor.NONE) {
                    intakeController.spinToTargetVelocity(REJECT_VELOCITY);
                    artifactTracker.setArtifact(slotPosition, leftColor);

                    if (isSpindexerFull()) {
                        intakeController.stop();
                        state = IntakeState.IDLE;
                    } else {
                        state = IntakeState.SPIN_SPINDEXER;
                    }
                } else {
                    artifactTracker.setArtifact(slotPosition, ArtifactColor.NONE);
                }
                break;
            case SPIN_SPINDEXER:
                if (spindexerController.isOnTarget()) {
                    double currentTarget = spindexerController.getTargetPosition();
                    double targetPosition = currentTarget + 120;
                    spindexerController.moveToPosition(targetPosition);
                    intakeController.spinToTargetVelocity(REJECT_VELOCITY);
                    state = IntakeState.REJECT_EXTRA_ARTIFACTS;
                }
                break;
            case REJECT_EXTRA_ARTIFACTS:
                if (spindexerController.isOnTarget()) {
                    state = IntakeState.INTAKE_ARTIFACTS;
                }
        }

        return Status.SUCCESS;

    }

    private boolean isSpindexerFull() {
        ArtifactColor[] currentPattern = artifactTracker.getCurrentPatternSnapshot();
        int numPurple = 0;
        int numGreen = 0;
        for (ArtifactColor artifact : currentPattern) {
            if (artifact == ArtifactColor.PURPLE) {
                numPurple++;
            } else if (artifact == GREEN) {
                numGreen++;
            }
        }

        return (numGreen + numPurple) == 3;
    }

}

