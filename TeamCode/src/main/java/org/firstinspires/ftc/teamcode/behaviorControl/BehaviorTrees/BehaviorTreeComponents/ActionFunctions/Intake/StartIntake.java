package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;

@Configurable
    public class StartIntake implements ActionFunction {

        Telemetry telemetry;
        IntakeController intakeController;
        Status lastStatus = Status.FAILURE;
        Status status;
    public static double INTAKE_VELOCITY = 320;
    private double lastVelocity = 0;

        public StartIntake(Telemetry telemetry, IntakeController intakeController) {
            this.telemetry = telemetry;
            this.intakeController = intakeController;
        }

        public Status perform(BlackBoard blackBoard) {
            if (lastStatus == Status.SUCCESS && INTAKE_VELOCITY == lastVelocity) {
                return Status.SUCCESS;
            }

            // Activate the intake mechanism
            intakeController.spinToTargetVelocity(INTAKE_VELOCITY);

            if(!intakeController.isOnTarget()){
                status = Status.RUNNING;
            } else {
                status = Status.SUCCESS;
            }

            lastVelocity = INTAKE_VELOCITY;
            lastStatus = status;
            return status;
        }
    }
