package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common.PauseAction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad.CheckTriggers;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad.HasTriggerNotTripped;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Gamepad.ResetTriggers;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Ramp.DeployRamp;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Ramp.IsRampDeployed;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Ramp.StoreRamp;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shooter.CalculateRPM;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.HasSpindexerSpun;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.SpinOnePosition;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.SwitchSpindexerState;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Conditional;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Selector;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Turret.TurretController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;

import java.util.Arrays;


public class ShootSubTree {
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected LinearOpMode opMode;
    /// Limelight
    protected LimeLightController limeLightController;
    ///
    protected SpindexerController spindexerController;
    ///
    protected IntakeController intakeController;
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
    private Node root;

    ///
    private BlackBoard blackBoard;

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
//        this.limeLightController = LimeLightController.getInstance();
        /// End Limelight

        /// Spindexer
        this.spindexerController = SpindexerController.getInstance();
        ///  End Spindexer

        /// Intake
        this.intakeController = IntakeController.getInstance();
        /// End Intake

        /// Ramp
        this.rampController = RampController.getInstance();
        /// End Ramp

        /// Turret
        this.turretController = TurretController.getInstance();
        /// End Turret

        /// Shooter
        this.shooterController = ShooterController.getInstance();
        /// End Shooter


        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        // Sequence
                        new Action(new CheckTriggers(telemetry), telemetry),
                        new Selector(
                                Arrays.asList(
                                        //Selector
                                        new Conditional(new HasTriggerNotTripped(telemetry)),
                                        new Sequence(
                                                Arrays.asList(
                                                        // Sequence
                                                        new Action(new CalculateRPM(telemetry, shooterController), telemetry),
                                                        new Selector(
                                                                Arrays.asList(
                                                                        // Selector
                                                                        new Conditional(new IsRampDeployed(telemetry, rampController)),
                                                                        new Sequence(
                                                                                Arrays.asList(
                                                                                        //Sequence
                                                                                        new Action(new DeployRamp(telemetry, rampController), telemetry),
                                                                                        new Action(new PauseAction(telemetry, 400), telemetry)
                                                                                ), telemetry
                                                                        )
                                                                ), telemetry
                                                        ),
                                                        // Sequence
//                                                        new Action(new SpinUpShooter(telemetry, shooterController), telemetry),
                                                        new Selector(
                                                                Arrays.asList(
                                                                        new Conditional(new HasSpindexerSpun(telemetry)),
                                                                        new Sequence(
                                                                                Arrays.asList(
                                                                                        new Action(new SpinOnePosition(telemetry, spindexerController), telemetry),
                                                                                        new Action(new SwitchSpindexerState(telemetry), telemetry)
                                                                                ), telemetry
                                                                        )
                                                                ), telemetry
                                                        ),
                                                        new Selector(
                                                                Arrays.asList(
                                                                        // Selector
//                                                                        new Conditional(new IsTriggerNOTHeld(telemetry)),
                                                                        new Sequence(
                                                                                Arrays.asList(
                                                                                        //Sequence
                                                                                        new Action(new StoreRamp(telemetry, rampController), telemetry),
                                                                                        new Action(new PauseAction(telemetry, 400), telemetry),
                                                                                        new Action(new ResetTriggers(telemetry), telemetry),
                                                                                        new Action(new SwitchSpindexerState(telemetry), telemetry)

                                                                                ), telemetry
                                                                        )
                                                                ), telemetry
                                                        )
                                                ), telemetry)
                                ), telemetry
                        )
                ), telemetry);

    }

    public Node getRoot() {
        // *****DON'T CLEAR THE HUBS HERE BECAUSE THIS IS A SUBTREE*****

        // Run the behavior tree
        return root;
    }
}