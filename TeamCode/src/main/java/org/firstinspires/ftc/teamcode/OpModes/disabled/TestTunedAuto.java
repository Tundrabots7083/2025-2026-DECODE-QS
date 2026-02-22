package org.firstinspires.ftc.teamcode.opModes.disabled;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Test Auto", group = "Examples")
public class TestTunedAuto extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(6.500, 8.500, Math.toRadians(90));
    private final Pose ctrlPoint1 = new Pose(41.000, 17.100);
    private final Pose endLine1Pose = new Pose(27.400, 44.600, Math.toRadians(180));

    private PathChain triangle;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("This will run in a roughly triangular shape, starting on the bottom-middle point.");
        telemetry.addLine("So, make sure you have enough space to the left, front, and right to run the OpMode.");
        telemetry.update();
        follower.update();
    }

    /**
     * Creates the PathChain for the "triangle".
     */
    @Override
    public void start() {
        follower.setStartingPose(startPose);

        triangle = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, ctrlPoint1, endLine1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endLine1Pose.getHeading())
                .build();

        follower.followPath(triangle);
    }
}