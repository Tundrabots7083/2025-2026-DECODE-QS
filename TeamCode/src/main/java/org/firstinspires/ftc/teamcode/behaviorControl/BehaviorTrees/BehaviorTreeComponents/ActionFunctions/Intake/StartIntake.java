package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;

    public class StartIntake implements ActionFunction {

        Telemetry telemetry;

        IntakeController intakeController;

        public StartIntake(Telemetry telemetry, IntakeController intakeController) {
            this.telemetry = telemetry;
            this.intakeController = intakeController;
        }

        public Status perform(BlackBoard blackBoard) {
            // Activate the intake mechanism
            intakeController.spinToTargetVelocity(320);

            if(!intakeController.isOnTarget()){
                return Status.RUNNING;
            } else {
                return Status.SUCCESS;
            }
        }
    }
