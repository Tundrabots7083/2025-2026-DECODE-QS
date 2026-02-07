package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Display;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimeLight.DetectMotifPattern;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;

public class DisplayMotif implements ActionFunction {

    Telemetry telemetry;

    LimeLightController limeLightController;

    Status status;

    int count = 0;



    public DisplayMotif(Telemetry telemetry, LimeLightController limeLightController) {
        this.telemetry = telemetry;
        this.limeLightController = limeLightController;
    }

    public Status perform(BlackBoard blackBoard) {


        if (blackBoard.getValue("Motif_Pattern") != null) {
            DetectMotifPattern.Pattern pattern = (DetectMotifPattern.Pattern) blackBoard.getValue("Motif_Pattern");
            telemetry.addData("Motif Pattern", pattern);
        } else if(count > 20){
            return Status.FAILURE;
        }

        count++;

        status = Status.RUNNING;

        return status;
    }
}