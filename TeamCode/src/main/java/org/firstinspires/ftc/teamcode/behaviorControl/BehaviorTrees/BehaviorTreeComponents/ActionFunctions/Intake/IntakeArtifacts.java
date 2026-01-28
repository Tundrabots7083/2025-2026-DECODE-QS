package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;

@Configurable
public class IntakeArtifacts implements ActionFunction {

    Telemetry telemetry;
    IntakeController intakeController;
    Status status;
    GamepadDelta gamepad_1_Delta;
    public static double INTAKE_VELOCITY = 400;
    private final double RETAIN_VELOCITY = 50;
    private boolean isIntaking = false;

    public IntakeArtifacts(Telemetry telemetry, IntakeController intakeController) {
        this.telemetry = telemetry;
        this.intakeController = intakeController;
    }

    public Status perform(BlackBoard blackBoard) {

        if (blackBoard.getValue("gamepad1Delta") != null) {
            gamepad_1_Delta = (GamepadDelta) blackBoard.getValue("gamepad1Delta");

            if (gamepad_1_Delta.dpadUpPressed) {
                // Activate the intake mechanism
                isIntaking = !isIntaking;
            }
            if (isIntaking) {
                intakeController.spinToTargetVelocity(INTAKE_VELOCITY);
            } else {
                intakeController.spinToTargetVelocity(RETAIN_VELOCITY);
            }
        }


        status = Status.SUCCESS;
        return status;
    }
}
