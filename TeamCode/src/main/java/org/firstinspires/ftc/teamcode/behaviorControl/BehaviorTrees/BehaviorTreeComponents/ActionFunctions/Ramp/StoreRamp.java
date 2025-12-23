package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Ramp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;

public class StoreRamp implements ActionFunction {

    Telemetry telemetry;
    RampController rampController;
    private final double STORED_RAMP = 0;
    Status lastStatus;

    public StoreRamp(Telemetry telemetry, RampController rampController) {
        this.telemetry = telemetry;
        this.rampController = rampController;
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        rampController.setTargetPosition(STORED_RAMP);
        lastStatus = Status.SUCCESS;
        return lastStatus;
    }
}