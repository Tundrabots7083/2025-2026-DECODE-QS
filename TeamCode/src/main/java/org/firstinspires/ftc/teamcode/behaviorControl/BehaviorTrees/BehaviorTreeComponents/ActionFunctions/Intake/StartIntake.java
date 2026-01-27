package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;

@Configurable
    public class StartIntake implements ActionFunction {

        Telemetry telemetry;
        IntakeController intakeController;
        Status lastStatus = Status.FAILURE;
        Status status;
    GamepadDelta gamepad_1_Delta;
    public static double INTAKE_VELOCITY = 320;

        public StartIntake(Telemetry telemetry, IntakeController intakeController) {
            this.telemetry = telemetry;
            this.intakeController = intakeController;
        }

        public Status perform(BlackBoard blackBoard) {
            if (lastStatus == Status.SUCCESS) {
                return lastStatus;
            }

            if (blackBoard.getValue("gamepad1Delta") != null) {
                gamepad_1_Delta = (GamepadDelta) blackBoard.getValue("gamepad1Delta");

                if (gamepad_1_Delta.aPressed) {
                    // Activate the intake mechanism
                    intakeController.spinToTargetVelocity(INTAKE_VELOCITY);
                }
            }

            if (!intakeController.isOnTarget() || intakeController.getTargetVelocity() != INTAKE_VELOCITY) {
                status = Status.RUNNING;
            } else {
                status = Status.SUCCESS;
            }

            lastStatus = status;
            return status;
        }
    }
