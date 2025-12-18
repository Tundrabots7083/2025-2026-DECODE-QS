package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Turret;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
        Pose3D pose = (Pose3D) blackBoard.getValue("CurrentPose");

        if(pose == null) {
            return Status.FAILURE;
        }

        double heading = pose.getOrientation().getYaw(AngleUnit.DEGREES);
        turretController.setTargetPosition(heading);

        return Status.SUCCESS;
    }
}