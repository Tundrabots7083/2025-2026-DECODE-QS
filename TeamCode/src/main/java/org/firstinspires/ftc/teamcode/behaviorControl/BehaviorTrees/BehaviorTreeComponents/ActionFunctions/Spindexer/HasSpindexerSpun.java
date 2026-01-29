package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Condition;

public class HasSpindexerSpun implements Condition {

    private Telemetry telemetry;

    public HasSpindexerSpun(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean check(BlackBoard blackBoard) {

        if (blackBoard.getValue("SpindexerHasSpun") != null) {
            return (boolean) blackBoard.getValue("SpindexerHasSpun");
        } else {
            return false;
        }

    }
}