package org.firstinspires.ftc.teamcode.opModes.test;


import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;


@Autonomous(name="TestPedro Pathing", group="test")
public class TestPedroPath extends LinearOpMode
{

    DriveTrainController driveTrainController;
    private final Pose startingPose = new Pose(48, 48, Math.toRadians(0));
    private final Pose centralPose = new Pose(72, 72);

    private PathChain moveToCentralPose;


    int count = 0;

    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(),telemetry);

    @Override
    public void runOpMode()
    {

        /// Drive Train
        this.driveTrainController = DriveTrainController.getInstance();
        this.driveTrainController.reset();
        this.driveTrainController.initialize(hardwareMap, startingPose);
        /// End Drivetrain

        joinedTelemetry.addData("DecodeScoringOpMode", "runOpMode started");
        joinedTelemetry.update();
        initialize(this);
        driveTrainController.followPath(moveToCentralPose, true);

        waitForStart();


        while (opModeIsActive())
        {
            count++;

            driveTrainController.update();

        }
    }


    private void initialize(LinearOpMode opMode){
        moveToCentralPose = driveTrainController.pathBuilder()
                .addPath(
                        new BezierLine(
                                startingPose,
                                centralPose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }


}


