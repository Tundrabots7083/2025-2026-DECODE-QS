package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;

import java.util.Objects;

public class CheckIfPoseIsOnRightSide implements ActionFunction {

    Status lastStatus = Status.FAILURE;

    public CheckIfPoseIsOnRightSide() {
    }

    public Status perform(BlackBoard blackBoard) {
        if (lastStatus == Status.SUCCESS || lastStatus == Status.FAILURE) {
            return lastStatus;
        }

        Pose robotPose = (Pose) blackBoard.getValue("CurrentPose");
        double x = robotPose.getX();

        String allianceColor = (String) blackBoard.getValue("AllianceColor");

        if (Objects.equals(allianceColor, "BLUE")) {
            if (x <= 72) {
                lastStatus = Status.SUCCESS;
                return Status.SUCCESS;
            } else {
                lastStatus = Status.FAILURE;
                return Status.FAILURE;
            }
        } else if (Objects.equals(allianceColor, "RED")) {
            if (x >= 72) {
                lastStatus = Status.SUCCESS;
                return Status.SUCCESS;
            } else {
                lastStatus = Status.FAILURE;
                return Status.FAILURE;
            }
        }

        return lastStatus = Status.RUNNING;
    }
}