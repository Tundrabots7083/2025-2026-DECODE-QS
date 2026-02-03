package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Turret;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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


    public TraverseTurretToRedGoal(Telemetry telemetry, TurretController turretController) {
        this.telemetry = telemetry;
        this.turretController = turretController;
        this.worldModel = (DecodeWorldModel) DecodeWorldModel.getInstance(telemetry);

    }

    public Status perform(BlackBoard blackBoard) {
        Pose3D robotPose = (Pose3D) blackBoard.getValue("CurrentPose");

        WorldObject goalObject = (WorldObject) worldModel.getValue("RedAllianceGoal");
        Pose3D goalPose = goalObject.pose3D;

        Position targetPosition = goalPose.getPosition();
        targetPosition.unit=DistanceUnit.INCH;

        if(robotPose == null) {
            return Status.FAILURE;
        }

        turretController.moveToTargetPosition(robotPose, targetPosition);

        return Status.SUCCESS;
    }
}