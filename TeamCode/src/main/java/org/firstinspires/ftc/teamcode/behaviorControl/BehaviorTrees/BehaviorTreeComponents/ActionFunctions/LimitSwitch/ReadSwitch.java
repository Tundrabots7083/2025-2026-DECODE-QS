package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimitSwitch;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.spindexerLimitSwitch.SpindexerLimitSwitchController;

@Configurable
public class ReadSwitch implements ActionFunction {

    Telemetry telemetry;
    SpindexerLimitSwitchController switchController;
    Status status;

    public ReadSwitch(Telemetry telemetry, SpindexerLimitSwitchController switchController) {
        this.telemetry = telemetry;
        this.switchController = switchController;
    }

    public Status perform(BlackBoard blackBoard) {

        // switch is pressed if value returned is HIGH or true.
        blackBoard.setValue("LimitSwitchState", switchController.getState());
        status = Status.SUCCESS;

        return status;
    }
}
