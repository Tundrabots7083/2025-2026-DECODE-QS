package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.worldModel.DecodeWorldModel;
import org.firstinspires.ftc.teamcode.worldModel.WorldObject;

public class CalculateRPM implements ActionFunction {
    Telemetry telemetry;
    ShooterController shooterController;
    DecodeWorldModel worldModel;
    protected Status lastStatus = Status.FAILURE;
    Pose robotPose;
    Position targetPose;

    double a = 0.1; //these represent the values
    double b = 0.1; //a,b,c, and d in the equation
    double c = 0.1; //velocity = a + bx + cx^2 + dx^3
    double d = 0.1;

    public CalculateRPM(Telemetry telemetry, ShooterController shooterController) {
        this.telemetry = telemetry;
        this.shooterController = shooterController;
        this.worldModel = DecodeWorldModel.getInstance(telemetry);
    }


    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
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

            blackBoard.setValue("TargetShooterRPM", calculateRPM(distanceToGoal));

        }

        blackBoard.setValue("TargetShooterRPM", 3500);
        status = Status.SUCCESS;

        return status;

    }

    private double calculateRPM(double distance) {
        return a
                + b * Math.pow(distance, 1)
                + c * Math.pow(distance, 2);
    }

    private double distanceToTarget(Pose current, Position target) {
        double dx = target.x - current.getX();
        double dy = target.y - current.getY();
        return Math.hypot(dx, dy);
    }
}
