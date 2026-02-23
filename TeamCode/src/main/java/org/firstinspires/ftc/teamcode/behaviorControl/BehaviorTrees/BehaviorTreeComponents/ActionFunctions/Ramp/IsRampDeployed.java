package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Ramp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Condition;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;

public class IsRampDeployed implements Condition {

    private Telemetry telemetry;
    private RampController rampController;

    public IsRampDeployed(Telemetry telemetry, RampController rampController) {
        this.telemetry = telemetry;
        this.rampController = rampController;
    }

    public boolean check(BlackBoard blackBoard) {
        return rampController.getPosition() > 100;
    }
}