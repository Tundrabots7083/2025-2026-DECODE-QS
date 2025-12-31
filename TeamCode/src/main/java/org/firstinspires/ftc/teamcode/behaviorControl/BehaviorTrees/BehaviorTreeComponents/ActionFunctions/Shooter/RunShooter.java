package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;


public class RunShooter implements ActionFunction {
    private ShooterController shooterController;
    private Telemetry telemetry;

    public RunShooter(Telemetry telemetry, ShooterController shooterController) {
        this.telemetry = telemetry;
        this.shooterController = shooterController;
    }

    @Override
    public Status perform(BlackBoard blackBoard) {
        shooterController.update();
        return Status.SUCCESS;
    }
}