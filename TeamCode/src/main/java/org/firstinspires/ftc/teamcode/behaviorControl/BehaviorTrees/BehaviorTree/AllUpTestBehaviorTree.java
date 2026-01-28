package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common.PauseAction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DistanceSensor.DetectArtifact;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad.ComputeGamepad_1_Delta;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad.ComputeGamepad_2_Delta;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad.ReadGamepadsSnapshot;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.RunIntake;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.StartIntake;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Conditional;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.spindexerLimitSwitch.SpindexerLimitSwitchController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.DistanceSensorController;

import java.util.Arrays;
import java.util.List;


public class AllUpTestBehaviorTree {
    private BehaviorTree tree;
    private Node root;
    private BlackBoard blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected LinearOpMode opMode;

    private final Pose startPose = new Pose(0, 0, 0);

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
    protected DistanceSensorController distanceSensorController;

    ///

    public AllUpTestBehaviorTree(LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;

        Init();
    }

    private void Init() {
        this.blackBoard = BlackBoard.getInstance(telemetry);
        this.blackBoard.reset();

        /// Spindexer
//        this.spindexerController = SpindexerController.getInstance();
//
//        this.spindexerController.reset();
//        this.spindexerController.initialize(hardwareMap, telemetry);
        ///  End Spindexer

        /// Ramp
//        this.rampController = RampController.getInstance();
//
//        this.rampController.reset();
//        this.rampController.initialize(hardwareMap, telemetry);
        /// End Ramp

        /// Switch
//        this.switchController = SpindexerLimitSwitchController.getInstance();
//
//        this.switchController.reset();
//        this.switchController.initialize(hardwareMap, telemetry);
        /// End Switch

        /// Intake
        this.intakeController = IntakeController.getInstance();

        this.intakeController.reset();
        this.intakeController.initialize(hardwareMap, telemetry);
        /// End Intake

        /// Shooter
//        this.shooterController = ShooterController.getInstance();
//
//        this.shooterController.reset();
//        this.shooterController.initialize(hardwareMap, telemetry);
        /// End Shooter

        /// Drivetrain
//        this.driveTrainController = DriveTrainController.getInstance();
//
//        this.driveTrainController.reset();
//        this.driveTrainController.initialize(hardwareMap, startPose);
        /// End Drivetrain

        /// DistanceSensor
        this.distanceSensorController = DistanceSensorController.getInstance();

        this.distanceSensorController.reset();
        this.distanceSensorController.initialize(hardwareMap, telemetry);
        /// End DistanceSensor

        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new RunIntake(telemetry, intakeController), telemetry),
//                        new Action(new RunShooter(telemetry, shooterController), telemetry),
//                        new Action(new RunDrivetrain(telemetry, driveTrainController), telemetry),
                        new Action(new ReadGamepadsSnapshot(telemetry, opMode), telemetry),
                        new Action(new ComputeGamepad_1_Delta(), telemetry),
                        new Action(new ComputeGamepad_2_Delta(), telemetry),
//                        new Action(new TeleOpDrive(telemetry, driveTrainController), telemetry),
//                        new CalibrateSpindexerSubTree(opMode, telemetry).getRoot(),
//                        new Action(new RunSpindexer(telemetry, spindexerController), telemetry),
//                        new Action(new SwitchToShootCoordinates(telemetry, spindexerController), telemetry),
//                        new Action(new SpinToZeroPosition(telemetry, spindexerController), telemetry),
                        new Conditional(new DetectArtifact(telemetry, distanceSensorController)),
                        new Action(new StartIntake(telemetry, intakeController), telemetry),
                        // Intake 3 Artifacts
                        new Action(new PauseAction(telemetry, 30000), telemetry)//,
//                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
//                        new Action(new PauseAction(telemetry, 2000), telemetry),
//                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
//                        new Action(new PauseAction(telemetry, 2000), telemetry),
//                        new Action(new RetainArtifacts(telemetry, intakeController), telemetry),
//                        new ShootSubTree(opMode, telemetry).getRoot(),
//                        new Action(new StoreRamp(telemetry, rampController), telemetry),
//                        new Action(new PauseAction(telemetry, 500), telemetry)
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

