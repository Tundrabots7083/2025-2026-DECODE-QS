package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.sensors.limeLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

public class DisplayPose implements ActionFunction {
    Telemetry telemetry;

    protected Status lastStatus = Status.FAILURE;


    public DisplayPose(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.init();
    }

    private void init() {

    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }



        Double x = (Double) blackBoard.getValue("AprilTagPose_X");
        Double y = (Double) blackBoard.getValue("AprilTagPose_Y");
        Double heading = (Double) blackBoard.getValue("AprilTagPose_HEADING");

        telemetry.addData("AprilTag X:", x)
                .addData("AprilTag Y:", y)
                .addData("AprilTag Heading:", heading);

        status = Status.SUCCESS;


        lastStatus = status;
        return status;
    }
}
