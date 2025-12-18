package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Ramp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;

public class DeployRamp implements ActionFunction {

    Telemetry telemetry;

    RampController rampController;

    public DeployRamp(Telemetry telemetry, RampController rampController) {
        this.telemetry = telemetry;
        this.rampController = rampController;
    }

    public Status perform(BlackBoard blackBoard) {
        rampController.deployRamp();
        return Status.SUCCESS;
    }
}