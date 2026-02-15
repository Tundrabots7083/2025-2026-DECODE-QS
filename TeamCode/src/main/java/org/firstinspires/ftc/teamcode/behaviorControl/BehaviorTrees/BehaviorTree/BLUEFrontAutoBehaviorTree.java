package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common.PauseAction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common.SetAllianceColorBlue;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common.SetAutonomous;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.BLUE.BLUEdriveToIntakePose1;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.BLUE.BLUEdriveToShootPoseFRONT;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.Relocalize;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.RunDrivetrain;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.UpdateBlackboardPose;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.WaitForDrivetrainToArrive;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.RunIntake;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.StopIntake;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimeLight.DetectRobotPose;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter.RunShooter;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.RunSpindexer;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Turret.TraverseTurretToBlueGoal;
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
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.spindexerLimitSwitch.SpindexerLimitSwitchController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.RightIntakeColorSensorController;

import java.util.Arrays;
import java.util.List;


public class BLUEFrontAutoBehaviorTree {
    private BehaviorTree tree;
    private Node root;
    private BlackBoard blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected LinearOpMode opMode;

    private final Pose startPose = new Pose(40, 90, Math.toRadians(320));

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

    ///
    protected LimeLightController limeLightController;

    ///

    public BLUEFrontAutoBehaviorTree(LinearOpMode opMode, Telemetry telemetry) {
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

        ///  Limelight
        this.limeLightController = LimeLightController.getInstance();

        this.limeLightController.reset();
        this.limeLightController.initialize(hardwareMap, telemetry);
        /// End Limelight

        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new SetAutonomous(), telemetry),
                        new Action(new SetAllianceColorBlue(), telemetry),
                        new Action(new RunShooter(telemetry, shooterController), telemetry),
                        new Action(new RunIntake(telemetry, intakeController), telemetry),
                        new Action(new RunSpindexer(telemetry, spindexerController), telemetry),
                        new Action(new RunDrivetrain(telemetry, driveTrainController), telemetry),
                        new Action(new DetectRobotPose(telemetry, limeLightController), telemetry),
                        new Action(new Relocalize(telemetry, driveTrainController), telemetry),
                        new Action(new UpdateBlackboardPose(telemetry, driveTrainController), telemetry),
                        new Action(new TraverseTurretToBlueGoal(telemetry, turretController), telemetry),
                        new Action(new BLUEdriveToShootPoseFRONT(telemetry, driveTrainController), telemetry),
                        new Action(new WaitForDrivetrainToArrive(telemetry, driveTrainController), telemetry),
                        new Action(new PauseAction(telemetry, 2500), telemetry),
                        new ShootSubTree(opMode, telemetry).getRoot(),
                        new Action(new BLUEdriveToIntakePose1(telemetry, driveTrainController), telemetry),
//                        new Action(new WaitForDrivetrainToArrive(telemetry, driveTrainController), telemetry),
//                        new Action(new IntakeArtifacts(telemetry, intakeController), telemetry),
//                        new Action(new BLUEDriveForwardToIntake1(telemetry, driveTrainController), telemetry),
//                        new Action(new DetectArtifactColor(telemetry, rightColorSensorController), telemetry),
//                        new Action(new TrackDetectedArtifact(telemetry, spindexerController, artifactTracker), telemetry),
//                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
//                        new Action(new DetectArtifactColor(telemetry, rightColorSensorController), telemetry),
//                        new Action(new TrackDetectedArtifact(telemetry, spindexerController, artifactTracker), telemetry),
//                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
//                        new Action(new DetectArtifactColor(telemetry, rightColorSensorController), telemetry),
//                        new Action(new TrackDetectedArtifact(telemetry, spindexerController, artifactTracker), telemetry),
//                                  new Action(new BLUEdriveToShootPoseREAR(telemetry, driveTrainController), telemetry),
//                        new Action(new BLUEdriveToShootPoseFRONT(telemetry, driveTrainController), telemetry),
//                        new Action(new SortArtifacts(telemetry, spindexerController), telemetry),
//                        new Action(new WaitForDrivetrainToArrive(telemetry, driveTrainController), telemetry),
//                                new ShootSubTree(opMode, telemetry).getRoot(),
//                        new Action(new BLUEdriveToIntakePose2(telemetry, driveTrainController), telemetry),
//                        new Action(new IntakeArtifacts(telemetry, intakeController), telemetry),
//                        new Action(new BLUEdriveToIntakePose2(telemetry, driveTrainController), telemetry),
//                        new Action(new DetectArtifactColor(telemetry, rightColorSensorController), telemetry),
//                        new Action(new TrackDetectedArtifact(telemetry, spindexerController, artifactTracker), telemetry),
//                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
//                        new Action(new DetectArtifactColor(telemetry, rightColorSensorController), telemetry),
//                        new Action(new TrackDetectedArtifact(telemetry, spindexerController, artifactTracker), telemetry),
//                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
//                        new Action(new DetectArtifactColor(telemetry, rightColorSensorController), telemetry),
//                        new Action(new TrackDetectedArtifact(telemetry, spindexerController, artifactTracker), telemetry)
                        new Action(new StopIntake(telemetry, intakeController), telemetry)
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

