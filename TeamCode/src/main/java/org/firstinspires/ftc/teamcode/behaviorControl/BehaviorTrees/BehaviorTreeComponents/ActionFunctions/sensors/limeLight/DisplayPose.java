package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.sensors.limeLight;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

public class DisplayPose implements ActionFunction {
    Telemetry telemetry;

    private Pose2D pose2D;
    private Pose pedroPose;

    protected Status lastStatus = Status.FAILURE;


    public DisplayPose(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.init();
    }

    private void init() {

    }

    public Status perform(BlackBoard blackBoard) {
        Status status;




        Double x = (Double) blackBoard.getValue("AprilTagPose_X");
        Double y = (Double) blackBoard.getValue("AprilTagPose_Y");
        Double heading = (Double) blackBoard.getValue("AprilTagPose_HEADING");
        Double PedroX = y + 72;
        Double PedroY = -x + 72;
        Double PedroHeading = (heading + 270) % 360;

        pedroPose = new Pose(x, y, heading, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        telemetry.addData("AprilTag X:", x)
                .addData("AprilTag Y:", y)
                .addData("AprilTag Heading:", heading)
                .addData("PedroPose X:", PedroX)
                .addData("PedroPose Y:", PedroY)
                .addData("PedroPose Heading:", PedroHeading)
                .addData("AutoConvert X:", pedroPose.getX())
                .addData("AutoConvert Y:", pedroPose.getY())
                .addData("AutoConvert Heading:", pedroPose.getHeading());

        status = Status.SUCCESS;


        lastStatus = status;
        return status;
    }
}
