package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Ramp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;

public class DeployRamp implements ActionFunction {

    Telemetry telemetry;
    RampController rampController;
    private final double DEPLOYED_RAMP = 100; //degrees at which the ramp is deployed
    Status lastStatus;

    public DeployRamp(Telemetry telemetry, RampController rampController) {
        this.telemetry = telemetry;
        this.rampController = rampController;
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        rampController.setTargetPosition(DEPLOYED_RAMP);
        lastStatus = Status.SUCCESS;
        return lastStatus;
    }
}