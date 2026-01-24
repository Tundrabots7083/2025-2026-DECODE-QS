package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;

public class CalculateRPM implements ActionFunction {
    Telemetry telemetry;
    ShooterController shooterController;
    protected Status lastStatus = Status.FAILURE;
    double distanceToShoot;

    double a = 0.1; //these represent the values
    double b = 0.1; //a,b,c, and d in the equation
    double c = 0.1; //velocity = a + bx + cx^2 + dx^3
    double d = 0.1;

    public CalculateRPM(Telemetry telemetry, ShooterController shooterController) {
        this.telemetry = telemetry;
        this.shooterController = shooterController;
    }


    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        if (blackBoard.getValue("DistanceToGoal") != null) {
            distanceToShoot = (double) blackBoard.getValue("DistanceToGoal");

            //Todo: actually calculate rpm
            blackBoard.setValue("TargetShooterRPM", calculateRPM());

        } else {
            status = Status.FAILURE;
        }

        blackBoard.setValue("TargetShooterRPM", 3500);
        status = Status.SUCCESS;

        return status;

    }

    private double calculateRPM() {
        return a
                + b * Math.pow(distanceToShoot, 1)
                + c * Math.pow(distanceToShoot, 2)
                + d * Math.pow(distanceToShoot, 3);

    }
}
