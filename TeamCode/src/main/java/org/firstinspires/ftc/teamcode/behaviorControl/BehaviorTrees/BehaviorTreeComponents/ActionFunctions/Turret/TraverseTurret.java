package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Turret;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Turret.TurretController;

public class TraverseTurret implements ActionFunction {

    Telemetry telemetry;

    TurretController turretController;

    public TraverseTurret(Telemetry telemetry, TurretController turretController) {
        this.telemetry = telemetry;
        this.turretController = turretController;
    }

    public Status perform(BlackBoard blackBoard) {
        Pose3D robotPose = (Pose3D) blackBoard.getValue("CurrentPose");

        //TODO: This is a placeholder Position;  Value should be acquired from blackboard or WorldModel
        Position targetPosition = new Position();
        targetPosition.x = 42.5;
        targetPosition.y=36.5;
        targetPosition.unit=DistanceUnit.INCH;
        ///////////////////////////////////////

        if(robotPose == null || targetPosition == null) {
            return Status.FAILURE;
        }

        turretController.moveToTargetPosition(robotPose, targetPosition);

        return Status.SUCCESS;
    }
}