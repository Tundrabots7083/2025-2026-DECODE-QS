package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimeLight;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.gamepad.GamepadSnapshot;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad.GamepadDelta;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;

import java.util.LinkedList;

public class DetectRobotPose implements ActionFunction {

    protected Status lastStatus = Status.FAILURE;
    Telemetry telemetry;
    LimeLightController limeLightController;
    boolean DpadUpIsPressed = false;

    // Moving average settings
    private static final int WINDOW_SIZE = 20;
    private final LinkedList<Pose> poseBuffer = new LinkedList<>();

    private Pose averagedPose = null;

    public DetectRobotPose(Telemetry telemetry, LimeLightController limeLightController) {
        this.telemetry = telemetry;
        this.limeLightController = limeLightController;
    }
    public Status perform(BlackBoard blackBoard) {
        Status status;
        boolean isAutonomous = (boolean) blackBoard.getValue("isAutonomous");

        if (lastStatus == Status.SUCCESS && isAutonomous) {
            return lastStatus;
        }

        // Read gamepad state (unchanged logic)
        if (blackBoard.getValue("gamepad2Snapshot") != null) {
            GamepadSnapshot gamepad2Snapshot =
                    (GamepadSnapshot) blackBoard.getValue("gamepad2Snapshot");
            DpadUpIsPressed = gamepad2Snapshot.dpadUp;
        }

        if (blackBoard.getValue("gamepad2Delta") != null) {
            GamepadDelta gamepad2Delta =
                    (GamepadDelta) blackBoard.getValue("gamepad2Delta");
            DpadUpIsPressed = gamepad2Delta.dpadUpPressed;
        }

        // --- REQUIRED BEHAVIOR 1 ---
        // Dpad NOT pressed → return null pose
        if (!DpadUpIsPressed) {
            poseBuffer.clear();  // optional but prevents stale data
            blackBoard.setValue("AprilTag_Pose", null);
            lastStatus = Status.SUCCESS;
            return lastStatus;
        }

        Pose currentPose = limeLightController.getCurrentRobotPose();

        if (currentPose == null) {
            blackBoard.setValue("AprilTag_Pose", null);
            lastStatus = Status.SUCCESS;
            return lastStatus;
        }

        // Add pose to buffer
        poseBuffer.add(currentPose);

        if (poseBuffer.size() > WINDOW_SIZE) {
            poseBuffer.removeFirst();
        }

        // --- REQUIRED BEHAVIOR 2 ---
        // Buffer NOT full → return null pose
        if (poseBuffer.size() < WINDOW_SIZE) {
            blackBoard.setValue("AprilTag_Pose", null);
            lastStatus = Status.SUCCESS;
            return lastStatus;
        }

        // --- REQUIRED BEHAVIOR 3 ---
        // Buffer full → compute & return averaged pose
        averagedPose = computeAveragePose();
        blackBoard.setValue("AprilTag_Pose", averagedPose);

        lastStatus = Status.SUCCESS;
        return lastStatus;
    }


    private Pose computeAveragePose() {

        double sumX = 0;
        double sumY = 0;
        double sumHeadingX = 0;
        double sumHeadingY = 0;

        for (Pose pose : poseBuffer) {
            sumX += pose.getX();
            sumY += pose.getY();

            double heading = pose.getHeading(); // degrees assumed
            sumHeadingX += Math.cos(Math.toRadians(heading));
            sumHeadingY += Math.sin(Math.toRadians(heading));
        }

        int size = poseBuffer.size();

        double avgX = sumX / size;
        double avgY = sumY / size;
        double avgHeading = Math.atan2(sumHeadingY / size, sumHeadingX / size);

        return new Pose(avgX, avgY, Math.toDegrees(avgHeading));
    }
}
