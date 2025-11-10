package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Common.EmptyAction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.actuators.driveTrain.MoveToShootingPosition;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;

import java.util.Arrays;
import java.util.List;


public class FrontZoneScoringBehaviorTree {
    private BehaviorTree tree;
    private Node root;
    private BlackBoard blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected LinearOpMode opMode;

    /// Drivetrain
    protected DriveTrainController driveTrainController;
    /// TODO: set the startingPose to the right value
    private final Pose startingPose = new Pose(10, 55, Math.toRadians(0));
    /// Emd Drivetrain

    /// Shoulder
    protected ShooterController shooterController;

    /// End Shoulder

    /// Arm
    protected IntakeController intakeController;

    /// End Arm

    public FrontZoneScoringBehaviorTree(LinearOpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

        Init();
    }
    private void Init() {
        this.blackBoard = BlackBoard.getInstance(telemetry);
        this.blackBoard.reset();

        /// Drive Train
        this.driveTrainController = DriveTrainController.getInstance();
        this.driveTrainController.reset();
        this.driveTrainController.initialize(hardwareMap, startingPose);
        /// End Drivetrain

        /// Shooter
        this.shooterController = ShooterController.getInstance();

        this.shooterController.reset();
        this.shooterController.initialize(hardwareMap, telemetry, this.opMode);

        /// End Shooter

        /// Intake
        this.intakeController = IntakeController.getInstance();

        this.intakeController.reset();
        this.intakeController.initialize(hardwareMap, telemetry, this.opMode);

        /// End Intake



        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new MoveToShootingPosition(telemetry, driveTrainController), telemetry),
                        new Action(new EmptyAction(telemetry, this.opMode), telemetry)
                ),telemetry);

        this.tree = new BehaviorTree(root, blackBoard);
    }

    public Status tick() {
        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        List<LynxModule> allHubs;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Run the behavior tree
        Status result = tree.tick();
        telemetry.addData("FrontZoneScoringBehaviorTree", "Run - Behavior tree result: %s",result);
        telemetry.update();

        return result;
    }
}

