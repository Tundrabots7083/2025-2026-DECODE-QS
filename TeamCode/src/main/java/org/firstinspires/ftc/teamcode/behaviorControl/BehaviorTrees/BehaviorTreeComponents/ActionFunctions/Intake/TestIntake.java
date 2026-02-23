package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;

@Configurable
public class TestIntake implements ActionFunction {

    Telemetry telemetry;
    IntakeController intakeController;
    Status lastStatus = Status.FAILURE;
    Status status;
    public static double velocity = 200;

    public TestIntake(Telemetry telemetry, IntakeController intakeController) {
        this.telemetry = telemetry;
        this.intakeController = intakeController;
    }

    public Status perform(BlackBoard blackBoard) {

        // Activate the intake mechanism
        intakeController.spinToTargetVelocity(velocity);

        status = Status.RUNNING;

        return status;
    }
}
