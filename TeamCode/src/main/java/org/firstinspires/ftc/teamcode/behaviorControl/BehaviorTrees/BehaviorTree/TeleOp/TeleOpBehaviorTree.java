package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common.SetTeleOp;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.RunDrivetrain;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.TeleOpDrive;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.UpdateBlackboardPose;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad.ComputeGamepad_1_Delta;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad.ComputeGamepad_2_Delta;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad.ReadGamepadsSnapshot;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.IntakeAction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.RunIntake;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter.RunShooter;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter.ShootAction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.RunSpindexer;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Turret.TraverseTurretToRedGoal;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.ArtifactTracker;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Turret.TurretController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.spindexerLimitSwitch.SpindexerLimitSwitchController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.RightIntakeColorSensorController;

import java.util.Arrays;
import java.util.List;


public class TeleOpBehaviorTree {
    private BehaviorTree tree;
    private Node root;
    private BlackBoard blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected LinearOpMode opMode;

    private final Pose startPose = new Pose(132, 12, 90);

    ///
    protected SpindexerController spindexerController;
    ///

    ///
    protected RampController rampController;
    ///

    ///
    protected SpindexerLimitSwitchController switchController;

    ///

    ///
    protected IntakeController intakeController;

    ///

    ///
    protected ShooterController shooterController;
    ///

    ///
    protected DriveTrainController driveTrainController;
    ///

    ///
    protected RightIntakeColorSensorController rightColorSensorController;
    ///

    ///
    protected ArtifactTracker artifactTracker;
    ///

    ///
    protected TurretController turretController;

    ///

    public TeleOpBehaviorTree(LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;

        Init();
    }

    private void Init() {
        this.blackBoard = BlackBoard.getInstance(telemetry);
        this.blackBoard.reset();

        /// Spindexer
        this.spindexerController = SpindexerController.getInstance();
        ///  End Spindexer

        /// Ramp
        this.rampController = RampController.getInstance();
        /// End Ramp

        /// Switch
        this.switchController = SpindexerLimitSwitchController.getInstance();
        /// End Switch

        /// Intake
        this.intakeController = IntakeController.getInstance();
        /// End Intake

        /// Shooter
        this.shooterController = ShooterController.getInstance();

        this.shooterController.reset();
        this.shooterController.initialize(hardwareMap, telemetry);
        /// End Shooter

        /// Drivetrain
        this.driveTrainController = DriveTrainController.getInstance();

        this.driveTrainController.reset();
        this.driveTrainController.initialize(hardwareMap, startPose);
        /// End Drivetrain

        /// Right Color Sensor
        this.rightColorSensorController = RightIntakeColorSensorController.getInstance();
        /// End Right Color Sensor

        /// Artifact Tracker
        this.artifactTracker = ArtifactTracker.getInstance();
        /// End Artifact Tracker

        /// Turret
        this.turretController = TurretController.getInstance();
        /// End Turret

        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new SetTeleOp(telemetry), telemetry),
                        new Action(new RunIntake(telemetry, intakeController), telemetry),
                        new Action(new RunShooter(telemetry, shooterController), telemetry),
                        new Action(new RunSpindexer(telemetry, spindexerController), telemetry),
                        new Action(new RunDrivetrain(telemetry, driveTrainController), telemetry),
                        new Action(new ReadGamepadsSnapshot(telemetry, opMode), telemetry),
                        new Action(new ComputeGamepad_1_Delta(), telemetry),
                        new Action(new ComputeGamepad_2_Delta(), telemetry),
                        new Action(new UpdateBlackboardPose(telemetry, driveTrainController), telemetry),
                        new Action(new TraverseTurretToRedGoal(telemetry, turretController), telemetry),
                        // Shoot if either trigger on GP1 is pressed
                        new Action(new ShootAction(telemetry), telemetry),
                        new Action(new TeleOpDrive(telemetry, driveTrainController), telemetry),
                        // Press dPadUp on GP1 to start or stop intake
                        new Action(new IntakeAction(telemetry), telemetry)
                ), telemetry);

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
        telemetry.addData("Display Test Result", result);
        telemetry.update();

        return result;
    }
}

