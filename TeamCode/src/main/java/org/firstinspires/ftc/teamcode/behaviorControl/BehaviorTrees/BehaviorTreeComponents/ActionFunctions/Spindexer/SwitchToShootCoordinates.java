package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;

@Configurable
public class SwitchToShootCoordinates implements ActionFunction {

    Telemetry telemetry;
    SpindexerController spindexerController;
    Status status;
    Status lastStatus = Status.FAILURE;

    public static double intakeDegreeOffset = -10.0;

    public SwitchToShootCoordinates(Telemetry telemetry, SpindexerController spindexerController) {
        this.telemetry = telemetry;
        this.spindexerController = spindexerController;
    }

    public Status perform(BlackBoard blackBoard) {

        spindexerController.setDegreeOffset(intakeDegreeOffset);

            return Status.SUCCESS;
    }
}
