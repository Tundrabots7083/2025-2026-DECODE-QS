package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad;


import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.gamepad.GamepadSnapshot;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;

@Configurable
public class ComputeGamepad_2_Delta implements ActionFunction {

    private final String currKey = "gamepad2Snapshot";
    private final String prevKey = "prevGamepad2Snapshot";
    private final String deltaKey = "gamepad2Delta";

    public ComputeGamepad_2_Delta() {
    }


    @Override
    public Status perform(BlackBoard blackBoard) {

        GamepadSnapshot curr = (GamepadSnapshot) blackBoard.getValue(currKey);
        GamepadSnapshot prev = (GamepadSnapshot) blackBoard.getValue(prevKey);

        // First tick initialization
        if (prev == null) {
            blackBoard.setValue(prevKey, curr);
            blackBoard.setValue(deltaKey, null);
            return Status.SUCCESS;
        }

        GamepadDelta delta = new GamepadDelta(prev, curr);

        blackBoard.setValue(deltaKey, delta);
        blackBoard.setValue(prevKey, curr);

        return Status.SUCCESS;
    }
}
