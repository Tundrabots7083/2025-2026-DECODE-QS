package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;

public class IntakeArtifacts implements ActionFunction {

    Telemetry telemetry;
    IntakeController intakeController;
    Status status;
    Status lastStatus = Status.FAILURE;
    private double INTAKE_VELOCITY = 200;

    public IntakeArtifacts(Telemetry telemetry, IntakeController intakeController) {
        this.telemetry = telemetry;
        this.intakeController = intakeController;
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        intakeController.spinToTargetVelocity(INTAKE_VELOCITY);

        status = Status.SUCCESS;
        lastStatus = status;
        return status;
    }
}
