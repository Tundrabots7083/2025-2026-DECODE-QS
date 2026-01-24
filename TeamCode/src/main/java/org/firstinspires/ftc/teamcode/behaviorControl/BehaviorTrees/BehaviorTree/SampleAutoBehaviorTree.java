package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree;

import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common.PauseAction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ArtifactTracker.MatchPatternToMotif;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ColorSensor.DetectArtifactColor;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.Relocalize;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.RetainArtifacts;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.RunIntake;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.StartIntake;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimeLight.DetectMotifPattern;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimeLight.DetectRobotPose;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter.RunShooter;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.SpinOnePosition;
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
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.IntakeColorSensorController;

import java.util.Arrays;
import java.util.List;


public class SampleAutoBehaviorTree {
    private BehaviorTree tree;
    private Node root;
    private BlackBoard blackBoard;
    protected JoinedTelemetry telemetry;
    protected HardwareMap hardwareMap;
    protected LinearOpMode opMode;

    /// Limelight
    protected LimeLightController limeLightController;
    ///

    ///
    protected SpindexerController spindexerController;
    ///

    ///
    protected IntakeController intakeController;
    ///

    ///
    protected RampController rampController;
    ///

    ///
    protected TurretController turretController;
    ///

    ///
    protected ShooterController shooterController;
    ///

    ///
    protected DriveTrainController driveTrainController;
    ///

    ///
    protected IntakeColorSensorController intakeColorSensorController;
    ///

    ///
    protected ArtifactTracker artifactTracker;

    ///


    public SampleAutoBehaviorTree(LinearOpMode opMode, JoinedTelemetry telemetry) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;

        Init();
    }

    private void Init() {
        this.blackBoard = BlackBoard.getInstance(telemetry);
        this.blackBoard.reset();

        /// Limelight
        this.limeLightController = LimeLightController.getInstance();

        this.limeLightController.reset();
        this.limeLightController.initialize(hardwareMap, telemetry);
        /// End Limelight

        /// Spindexer
        this.spindexerController = SpindexerController.getInstance();

        this.spindexerController.reset();
        this.spindexerController.initialize(hardwareMap, telemetry);
        ///  End Spindexer

        /// Intake
        this.intakeController = IntakeController.getInstance();

        this.intakeController.reset();
        this.intakeController.initialize(hardwareMap, telemetry);
        /// End Intake

        /// Ramp
        this.rampController = RampController.getInstance();

        this.rampController.reset();
        this.rampController.initialize(hardwareMap, telemetry);
        /// End Ramp

        /// Turret
        this.turretController = TurretController.getInstance();

        this.turretController.reset();
        this.turretController.initialize(hardwareMap, telemetry);
        /// End Turret

        /// Shooter
        this.shooterController = ShooterController.getInstance();

        this.shooterController.reset();
        this.shooterController.initialize(hardwareMap, telemetry);
        /// End Shooter

        /// Drivetrain
        this.driveTrainController = DriveTrainController.getInstance();

        this.driveTrainController.reset();
        this.driveTrainController.initialize(hardwareMap, new Pose(72, 72, 0));
        /// End Drivetrain

        /// Intake Color Sensor
        this.intakeColorSensorController = IntakeColorSensorController.getInstance();

        this.intakeColorSensorController.reset();
        this.intakeColorSensorController.initialize(hardwareMap, telemetry);
        /// End Intake Color Sensor

        /// Artifact Tracker
        this.artifactTracker = ArtifactTracker.getInstance();

        this.artifactTracker.reset();
        this.artifactTracker.initialize(hardwareMap, telemetry);
        /// End Artifact Tracker


        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new RunShooter(telemetry, shooterController), telemetry),
                        new Action(new RunIntake(telemetry, intakeController), telemetry),
                        new Action(new DetectMotifPattern(telemetry, limeLightController), telemetry),
                        new Action(new MatchPatternToMotif(telemetry, spindexerController, artifactTracker), telemetry),
                        new Action(new DetectRobotPose(telemetry, limeLightController), telemetry),
                        new Action(new Relocalize(telemetry, driveTrainController), telemetry),
                        // drive to shooting position
                        // simultaneously sort artifacts
                        new ShootSubTree(opMode, telemetry).getRoot(),
                        // drive to intake position
                        new Action(new StartIntake(telemetry, intakeController), telemetry),
                        // drive through pile
                        new Action(new DetectArtifactColor(telemetry, intakeColorSensorController), telemetry),
                        //conditional to check if took in an artifact
                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
                        //conditional to check if took in an artifact
                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
                        //conditional to check if took in an artifact
                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
                        //conditional to check if took in an artifact
                        new Action(new RetainArtifacts(telemetry, intakeController), telemetry),
                        // drive to shoot and sort
                        new ShootSubTree(opMode, telemetry).getRoot(),
                        // drive to park position
                        new Action(new PauseAction(telemetry, 2000), telemetry)
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

