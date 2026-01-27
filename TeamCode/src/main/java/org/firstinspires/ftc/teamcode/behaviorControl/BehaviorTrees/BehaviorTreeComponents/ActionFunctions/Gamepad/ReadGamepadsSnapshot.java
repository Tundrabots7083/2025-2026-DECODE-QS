package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.gamepad.GamepadSnapshot;

@Configurable
public class ReadGamepadsSnapshot implements ActionFunction {

    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;

    public ReadGamepadsSnapshot(
            Telemetry telemetry,
            OpMode opMode
    ) {
        this.telemetry = telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
    }

    @Override
    public Status perform(BlackBoard blackBoard) {

        blackBoard.setValue("gamepad1Snapshot", new GamepadSnapshot(gamepad1));
        blackBoard.setValue("gamepad2Snapshot", new GamepadSnapshot(gamepad2));

        return Status.SUCCESS;
    }
}
