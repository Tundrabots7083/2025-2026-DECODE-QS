package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Ramp.DeployRamp;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter.CalculateRPM;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter.SpinUpShooter;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.SpinOnePosition;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Turret.TurretController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;

import java.util.Arrays;


public class ShootSubTree {
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
    protected ShooterController shooterController;

    ///


    public ShootSubTree(LinearOpMode opMode, Telemetry telemetry) {
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


        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new CalculateRPM(telemetry, shooterController), telemetry),
                        new Action(new SpinUpShooter(telemetry, shooterController), telemetry),
                        new Action(new DeployRamp(telemetry, rampController), telemetry),
                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
                        new Action(new SpinUpShooter(telemetry, shooterController), telemetry),
                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
                        new Action(new SpinUpShooter(telemetry, shooterController), telemetry),
                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry)
                ), telemetry);

    }

    public Node getRoot() {
        // *****DON'T CLEAR THE HUBS HERE BECAUSE THIS IS A SUBTREE*****

        // Run the behavior tree
        return root;
    }
}