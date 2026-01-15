package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimitSwitch;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.spindexerLimitSwitch.SpindexerLimitSwitchController;

@Configurable
public class TestSwitch implements ActionFunction {

    Telemetry telemetry;
    SpindexerLimitSwitchController switchController;
    Status lastStatus = Status.FAILURE;
    Status status;

    public TestSwitch(Telemetry telemetry, SpindexerLimitSwitchController switchController) {
        this.telemetry = telemetry;
        this.switchController = switchController;
    }

    public Status perform(BlackBoard blackBoard) {

        telemetry.addData("Switch State", switchController.getState());
        status = Status.RUNNING;

        return status;
    }
}
