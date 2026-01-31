package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake;

import static org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor.GREEN;

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


public class IntakeAction implements ActionFunction {
    protected Status lastStatus = Status.FAILURE;
    Telemetry telemetry;
    IntakeController intakeController;
    RampController rampController;
    SpindexerController spindexerController;
    LeftIntakeColorSensorController leftColorSensorController;
    RightIntakeColorSensorController rightColorSensorController;
    ArtifactTracker artifactTracker;
    IntakeState state = IntakeState.IDLE;
    boolean wasDpadUpPressed = false;
    double RETAIN_VELOCITY = 80;
    double INTAKE_VELOCITY = 380;

    public IntakeAction(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.intakeController = IntakeController.getInstance();
        this.rampController = RampController.getInstance();
        this.spindexerController = SpindexerController.getInstance();
        this.leftColorSensorController = LeftIntakeColorSensorController.getInstance();
        this.rightColorSensorController = RightIntakeColorSensorController.getInstance();
        this.artifactTracker = ArtifactTracker.getInstance();
    }


    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (blackBoard.getValue("gamepad1Delta") != null) {
            GamepadDelta gamepad1Delta = (GamepadDelta) blackBoard.getValue("gamepad1Delta");
            wasDpadUpPressed = gamepad1Delta.dpadUpPressed;
        } else {
//            return Status.FAILURE;
        }

        switch (state) {
            case IDLE:
                if (wasDpadUpPressed) {
                    state = IntakeState.STORE_RAMP;
                    return Status.SUCCESS;
                } else {
//                    return Status.FAILURE;
                }
                break;
            case STORE_RAMP:
                if (wasDpadUpPressed) {
                    intakeController.spinToTargetVelocity(RETAIN_VELOCITY);
                    state = IntakeState.IDLE;
                    return Status.SUCCESS;
                }
                if (rampController.isDeployed()) {
                    rampController.store();
                } else {
                    state = IntakeState.INTAKE_ARTIFACTS;
                }
                break;
            case INTAKE_ARTIFACTS:
                if (wasDpadUpPressed) {
                    intakeController.spinToTargetVelocity(RETAIN_VELOCITY);
                    state = IntakeState.IDLE;
                    return Status.SUCCESS;
                }
//                intakeController.spinToTargetVelocity(INTAKE_VELOCITY);
                if (spindexerController.isOnTarget()) {
                    state = IntakeState.CHECK_SLOT;
                }
                break;
            case CHECK_SLOT:
                if (wasDpadUpPressed) {
                    intakeController.spinToTargetVelocity(RETAIN_VELOCITY);
                    state = IntakeState.IDLE;
                    return Status.SUCCESS;
                }
                ArtifactColor rightColor = rightColorSensorController.getColor();
                ArtifactColor leftColor = leftColorSensorController.getColor();

                if (leftColor != ArtifactColor.NONE
                        && rightColor == leftColor
                ) {
                    int slotPosition = spindexerController.getSlotPosition();
                    artifactTracker.setArtifact(slotPosition, rightColor);
                    state = IntakeState.CHECK_FULL;
                } else if (leftColor == ArtifactColor.PURPLE || rightColor == ArtifactColor.PURPLE) {
                    int slotPosition = spindexerController.getSlotPosition();
                    artifactTracker.setArtifact(slotPosition, ArtifactColor.PURPLE);
                    state = IntakeState.CHECK_FULL;
                } else if (leftColor == ArtifactColor.NONE || rightColor == ArtifactColor.NONE) {
                    int slotPosition = spindexerController.getSlotPosition();
                    artifactTracker.setArtifact(slotPosition, ArtifactColor.NONE);
                }
                break;
            case CHECK_FULL:
                if (wasDpadUpPressed) {
                    intakeController.spinToTargetVelocity(RETAIN_VELOCITY);
                    state = IntakeState.IDLE;
                    return Status.SUCCESS;
                }
                if (spindexerController.isOnTarget()) {
                    if (isSpindexerFull()) {
                        intakeController.spinToTargetVelocity(RETAIN_VELOCITY);
                        state = IntakeState.IDLE;
                    } else {
                        state = IntakeState.SPIN_SPINDEXER;
                    }
                }
                break;
            case SPIN_SPINDEXER:
                double currentTarget = spindexerController.getTargetPosition();
                double targetPosition = currentTarget + 120;
                spindexerController.moveToPosition(targetPosition);
                state = IntakeState.INTAKE_ARTIFACTS;
                break;
        }

        telemetry.addData("[INTAKE ACTION] Atrifact Tracker 1", artifactTracker.getArtifact(0));
        telemetry.addData("[INTAKE ACTION] Atrifact Tracker 2", artifactTracker.getArtifact(1));
        telemetry.addData("[INTAKE ACTION] Atrifact Tracker 3", artifactTracker.getArtifact(2));
        telemetry.addData("[INTAKE ACTION] Current Slot", spindexerController.getSlotPosition());
        telemetry.addData("[INTAKE ACTION] Current Spindex Position", spindexerController.getPosition());
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


enum IntakeState {
    IDLE,
    INTAKE_ARTIFACTS,
    STORE_RAMP,
    CHECK_SLOT,
    SPIN_SPINDEXER,
    CHECK_FULL
}

