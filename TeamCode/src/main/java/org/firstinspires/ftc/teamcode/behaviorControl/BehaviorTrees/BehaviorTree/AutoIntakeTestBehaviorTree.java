package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common.PauseAction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common.SetAutonomous;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ArtifactTracker.TrackDetectedArtifact;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.ColorSensor.DetectArtifactColor;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.RED.REDriveForwardToIntakeGOAL;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.DriveTrain.RunDrivetrain;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.IntakeArtifacts;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.RunIntake;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.StopIntake;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.RunSpindexer;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.SpinOnePosition;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.SpinToZeroPosition;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.SwitchToShootCoordinates;
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
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.spindexerLimitSwitch.SpindexerLimitSwitchController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.RightIntakeColorSensorController;

import java.util.Arrays;
import java.util.List;


public class AutoIntakeTestBehaviorTree {
    private BehaviorTree tree;
    private Node root;
    private BlackBoard blackBoard;
    protected Telemetry telemetry;
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
    protected SpindexerLimitSwitchController switchController;
    ///


    ///
    protected RightIntakeColorSensorController rightColorSensorController;
    ///

    ///
    protected DriveTrainController driveTrainController;
    ///


    ///
    protected ArtifactTracker artifactTracker;
    ///

    private final Pose startPose = new Pose(100, 35, Math.toRadians(0)); // Start Pose of our robot.

    public AutoIntakeTestBehaviorTree(LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;

        Init();
    }

    private void Init() {
        this.blackBoard = BlackBoard.getInstance(telemetry);
        this.blackBoard.reset();

       /* /// Limelight
        this.limeLightController = LimeLightController.getInstance();

        this.limeLightController.reset();
        this.limeLightController.initialize(hardwareMap, telemetry);
        /// End Limelight
*/
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

        /// Switch
        this.switchController = SpindexerLimitSwitchController.getInstance();

        this.switchController.reset();
        this.switchController.initialize(hardwareMap, telemetry);
        /// End Switch

        /// ColorSensor
        this.rightColorSensorController = RightIntakeColorSensorController.getInstance();

        this.rightColorSensorController.reset();
        this.rightColorSensorController.initialize(hardwareMap, telemetry);

        /// End ColorSensor

        /// Drivetrain
        this.driveTrainController = DriveTrainController.getInstance();

        this.driveTrainController.reset();
        this.driveTrainController.initialize(hardwareMap, startPose);
        /// End Drivetrain

        /// Artifact Tracker
        this.artifactTracker = ArtifactTracker.getInstance();

        this.artifactTracker.reset();
        this.artifactTracker.initialize(telemetry);
        /// End Artifact Tracker

        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new SetAutonomous(), telemetry),
                        new Action(new RunDrivetrain(telemetry, driveTrainController), telemetry),
                        new Action(new RunIntake(telemetry, intakeController), telemetry),
                        new CalibrateSpindexerSubTree(opMode, telemetry).getRoot(),
                        new Action(new PauseAction(telemetry, 50), telemetry),
                        new Action(new RunSpindexer(telemetry, spindexerController), telemetry),
                        new Action(new SwitchToShootCoordinates(telemetry, spindexerController), telemetry),
                        new Action(new SpinToZeroPosition(telemetry, spindexerController), telemetry),
                        new Action(new TrackDetectedArtifact(telemetry, spindexerController, artifactTracker), telemetry),
                        new Action(new REDriveForwardToIntakeGOAL(telemetry, driveTrainController), telemetry),
                        new Action(new IntakeArtifacts(telemetry, intakeController), telemetry),
                        new Action(new DetectArtifactColor(telemetry, rightColorSensorController), telemetry),
                        new Action(new StopIntake(telemetry, intakeController), telemetry),
                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
                        new Action(new IntakeArtifacts(telemetry, intakeController), telemetry),
                        new Action(new DetectArtifactColor(telemetry, rightColorSensorController), telemetry),
                        new Action(new StopIntake(telemetry, intakeController), telemetry),
                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
                        new Action(new IntakeArtifacts(telemetry, intakeController), telemetry),
                        new Action(new DetectArtifactColor(telemetry, rightColorSensorController), telemetry)
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

