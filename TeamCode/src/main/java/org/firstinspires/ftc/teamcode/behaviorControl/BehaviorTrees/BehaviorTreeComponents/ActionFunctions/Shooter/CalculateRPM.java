package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Turret.TurretController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.worldModel.DecodeWorldModel;
import org.firstinspires.ftc.teamcode.worldModel.WorldObject;

public class CalculateRPM implements ActionFunction {
    Telemetry telemetry;
    ShooterController shooterController;
    TurretController turretController;
    DecodeWorldModel worldModel;
    protected Status lastStatus = Status.FAILURE;
    Pose robotPose;
    Position targetPose;

    double a = 2101; //these represent the values
    double b = -3.79; //a,b,c, and d in the equation
    double c = 0.077; //velocity = a + bx + cx^2 + dx^3

    double A = 0.732;
    double B = -3.32E-03;

    public CalculateRPM(Telemetry telemetry, ShooterController shooterController, TurretController turretController) {
        this.telemetry = telemetry;
        this.shooterController = shooterController;
        this.turretController = turretController;
        this.worldModel = DecodeWorldModel.getInstance(telemetry);
    }


    public Status perform(BlackBoard blackBoard) {
        Status status;
        boolean isAutonomous = (boolean) blackBoard.getValue("isAutonomous");

        if (lastStatus == Status.SUCCESS && isAutonomous) {
            return lastStatus;
        }

        if (blackBoard.getValue("AllianceColor") != null) {
            String allianceColor = (String) blackBoard.getValue("AllianceColor");
            WorldObject goalObject;
            if ("RED".equals(allianceColor)) {
                goalObject = worldModel.getValue("RedAllianceGoal");
            } else {
                goalObject = worldModel.getValue("BlueAllianceGoal");
            }
            targetPose = goalObject.position;
        }

        if (blackBoard.getValue("CurrentPose") != null) {
            robotPose = (Pose) blackBoard.getValue("CurrentPose");

            double distanceToGoal = distanceToTarget(robotPose, targetPose);

            telemetry.addData("[CalculateRPM] Distance", distanceToGoal);
            telemetry.addData("[CalculateRPM] x", robotPose.getX());
            telemetry.addData("[CalculateRPM] y", robotPose.getY());

            blackBoard.setValue("TargetShooterRPM", calculateRPM(distanceToGoal));
            blackBoard.setValue("SpindexerPower", calculateSpindexPower(distanceToGoal));

            lastStatus = Status.SUCCESS;
            return lastStatus;


        }

        if (!isAutonomous) {
            status = Status.SUCCESS;
        } else {
            telemetry.addData("RUNNNNNNNNNNNNNNNNNNNNNNNNNn", "RJRJRJ");
            status = Status.RUNNING;
        }

        return status;

    }

    private double calculateRPM(double distance) {
        double baseRpm = a
                + b * Math.pow(distance, 1)
                + c * Math.pow(distance, 2);
        double headingScalar;
        if (turretController.getPosition() > 180) {
            headingScalar = (turretController.getPosition() - 360) / 180;
        } else {
            headingScalar = turretController.getPosition() / 180;
        }
        return baseRpm;//+ 100 * headingScalar;
    }

    private double calculateSpindexPower(double distance) {
        double spindexPower = A
                + B * Math.pow(distance, 1);
        return spindexPower;
    }

    private double distanceToTarget(Pose current, Position target) {
        double dx = target.x - current.getX();
        double dy = target.y - current.getY();
        return Math.hypot(dx, dy);
    }
}