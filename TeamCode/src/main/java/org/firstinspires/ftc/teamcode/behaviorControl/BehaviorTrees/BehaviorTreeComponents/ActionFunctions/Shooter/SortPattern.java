package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter;

import static org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor.GREEN;
import static org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor.PURPLE;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimeLight.DetectMotifPattern;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.ArtifactTracker;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor;

enum SortingState {
    IDLE,
    TAKE_INVENTORY,
    CALCULATE_SPINS,
    RETRACT,
    SPIN_SPINDEXER,
    WAIT_FOR_SPINDEXER
}

@Configurable
public class SortPattern implements ActionFunction {
    protected Status lastStatus = Status.FAILURE;
    Telemetry telemetry;
    ShooterController shooterController;
    RampController rampController;
    SpindexerController spindexerController;
    ArtifactTracker artifactTracker;
    IntakeController intakeController;
    SortingState state = SortingState.IDLE;
    boolean wasAPressed = false;
    boolean wasBPressed = false;
    boolean wasXPressed = false;
    DetectMotifPattern.Pattern motifPattern;
    boolean cancelConsumed = false;
    int timesSpun = 0;

    private ArtifactColor[] currentPattern;
    private ArtifactColor[] targetPattern;
    private int numPurple = 0;
    private int numGreen = 0;
    private boolean isNotFull = true;
    LinearOpMode opMode;
    private int spinsToPattern = 0;

    public static double shooterRPM = 3500;

    public SortPattern(Telemetry telemetry, LinearOpMode opMode) {
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.shooterController = ShooterController.getInstance();
        this.rampController = RampController.getInstance();
        this.spindexerController = SpindexerController.getInstance();
        this.artifactTracker = ArtifactTracker.getInstance();
        this.intakeController = IntakeController.getInstance();
    }


    public Status perform(BlackBoard blackBoard) {

        if (blackBoard.getValue("gamepad2Delta") != null) {
            GamepadDelta gamepad1Delta = (GamepadDelta) blackBoard.getValue("gamepad1Delta");
            wasAPressed = gamepad1Delta.aPressed;
            wasBPressed = gamepad1Delta.bPressed;
            wasXPressed = gamepad1Delta.xPressed;
        }

        if (blackBoard.getValue("TargetShooterRPM") != null) {
            shooterRPM = (double) blackBoard.getValue("TargetShooterRPM");
//            telemetry.addData("[SHOOT ACTION] shooterRPM",shooterRPM);
        }

//        telemetry.addData("[SHOOTACTION] Front Speed", shooterController.getFrontCurrentVelocity());
//        telemetry.addData("[SHOOTACTION] Reare Speed", shooterController.getRearCurrentVelocity());

        switch (state) {
            case IDLE:
                timesSpun = 0;
                if (!cancelConsumed) {
                    if (wasAPressed) {
                        state = SortingState.TAKE_INVENTORY;
                        motifPattern = DetectMotifPattern.Pattern.PPG;
                        intakeController.spinToTargetVelocity(200);
                        break;
                    } else if (wasBPressed) {
                        state = SortingState.TAKE_INVENTORY;
                        motifPattern = DetectMotifPattern.Pattern.PGP;
                        intakeController.spinToTargetVelocity(200);
                        break;
                    } else if (wasXPressed) {
                        state = SortingState.TAKE_INVENTORY;
                        motifPattern = DetectMotifPattern.Pattern.GPP;
                        intakeController.spinToTargetVelocity(200);
                        break;
                    }
                } else {
                    cancelConsumed = false;
                }
                return Status.SUCCESS;
            case TAKE_INVENTORY:
                currentPattern = artifactTracker.getCurrentPatternSnapshot();
                countArtifacts();

                if (isNotFull) {
                    opMode.gamepad2.rumbleBlips(3);
                    break;
                } else if (numPurple != 2) {
                    opMode.gamepad2.rumble(500);
                    break;
                }

                if (motifPattern == null) {
                    break;
                } else if (targetPattern == null) {
                    createTargetPattern();
                }

                if (currentPattern == targetPattern) {
                    spinsToPattern = 0;
                } else if (currentPattern == rotateArray(targetPattern)) {
                    spinsToPattern = 1;
                } else if (currentPattern == rotateArray(rotateArray(targetPattern))) {
                    spinsToPattern = 2;
                }

                break;
            case RETRACT:
                if (rampController.isDeployed()) {
                    rampController.store();
                } else {
                    state = SortingState.SPIN_SPINDEXER;
                }
                break;
            case SPIN_SPINDEXER:
                double targetPosition = spindexerController.getTargetPosition();
                switch (spinsToPattern) {
                    case 0:
                        break;
                    case 1:
                        spindexerController.moveToPosition(targetPosition + 120);
                        break;
                    case 2:
                        spindexerController.moveToPosition(targetPosition + 240);
                        break;
                }
                state = SortingState.WAIT_FOR_SPINDEXER;
                break;
            case WAIT_FOR_SPINDEXER:
                if (spindexerController.isOnTarget()) {
                    state = SortingState.IDLE;
                }
                break;
        }

        return Status.SUCCESS;

    }


    private void countArtifacts() {
        for (ArtifactColor artifact : currentPattern) {
            if (artifact == ArtifactColor.PURPLE) {
                numPurple++;
            } else if (artifact == GREEN) {
                numGreen++;
            }
        }

        isNotFull = numGreen + numPurple != 3;
    }

    private void createTargetPattern() {
        switch (motifPattern) {
            case PPG:
                targetPattern = new ArtifactColor[]{PURPLE, PURPLE, GREEN};
                break;
            case GPP:
                targetPattern = new ArtifactColor[]{GREEN, PURPLE, PURPLE};
                break;
            case PGP:
                targetPattern = new ArtifactColor[]{PURPLE, GREEN, PURPLE};
                break;
        }
    }

    private ArtifactColor[] rotateArray(ArtifactColor[] arrayToRotate) {
        ArtifactColor[] arrayToReturn = new ArtifactColor[3];

        //shifts array to the right by 1 slot
        for (int i = 0; i <= 2; i++) {
            arrayToReturn[(i + 1) % 3] = arrayToRotate[i];
        }

        return arrayToReturn;
    }
}