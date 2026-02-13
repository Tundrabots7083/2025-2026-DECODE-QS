package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Turret;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Turret.TurretController;
import org.firstinspires.ftc.teamcode.worldModel.DecodeWorldModel;
import org.firstinspires.ftc.teamcode.worldModel.WorldObject;

public class TraverseTurretToRedGoal implements ActionFunction {

    Telemetry telemetry;

    TurretController turretController;
    private final DecodeWorldModel worldModel;
    Status lastStatus = Status.FAILURE;


    public TraverseTurretToRedGoal(Telemetry telemetry, TurretController turretController) {
        this.telemetry = telemetry;
        this.turretController = turretController;
        this.worldModel = DecodeWorldModel.getInstance(telemetry);

    }

    public Status perform(BlackBoard blackBoard) {
        boolean isAutonomous = (boolean) blackBoard.getValue("isAutonomous");

        if (lastStatus == Status.SUCCESS && isAutonomous) {
            return lastStatus;
        }

        Pose robotPose = (Pose) blackBoard.getValue("CurrentPose");

        WorldObject goalObject = worldModel.getValue("RedAllianceGoal");

        Position targetPosition = goalObject.position;

        if(robotPose == null) {
            return Status.FAILURE;
        }

        if (distanceToTarget(robotPose, targetPosition) > 100) {
            turretController.moveToTargetPosition(robotPose, targetPosition, 3);
        } else {
            turretController.moveToTargetPosition(robotPose, targetPosition, 5);
        }
//        telemetry.addData("[TraverseTurret] TurretPose", turretController.getPosition() - 122);


        return Status.SUCCESS;
    }


    private double distanceToTarget(Pose current, Position target) {
        double dx = target.x - current.getX();
        double dy = target.y - current.getY();
        return Math.hypot(dx, dy);
    }
}