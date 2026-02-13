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

public class TraverseTurretToBlueGoal implements ActionFunction {

    Telemetry telemetry;

    TurretController turretController;
    private final DecodeWorldModel worldModel;


    public TraverseTurretToBlueGoal(Telemetry telemetry, TurretController turretController) {
        this.telemetry = telemetry;
        this.turretController = turretController;
        this.worldModel = DecodeWorldModel.getInstance(telemetry);

    }

    public Status perform(BlackBoard blackBoard) {
        Pose robotPose = (Pose) blackBoard.getValue("CurrentPose");

        WorldObject goalObject = worldModel.getValue("BlueAllianceGoal");

        Position targetPosition = goalObject.position;

        if(robotPose == null) {
            return Status.FAILURE;
        }

        turretController.moveToTargetPosition(robotPose, targetPosition, 5);

        return Status.SUCCESS;
    }
}