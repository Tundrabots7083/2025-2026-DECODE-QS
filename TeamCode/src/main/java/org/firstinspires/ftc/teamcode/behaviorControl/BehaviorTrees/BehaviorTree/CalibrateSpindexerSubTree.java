package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.AA_Common.PauseAction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.RetainArtifacts;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Intake.StopIntake;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimitSwitch.ReadSwitch;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Ramp.StoreRamp;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.RunSpindexerToReference;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Ramp.RampController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.intake.IntakeController;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.spindexerLimitSwitch.SpindexerLimitSwitchController;

import java.util.Arrays;


public class CalibrateSpindexerSubTree {
    private BehaviorTree tree;
    private Node root;
    private BlackBoard blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected LinearOpMode opMode;

    ///
    protected SpindexerController spindexerController;
    ///

    ///
    protected SpindexerLimitSwitchController switchController;

    ///

    ///
    protected RampController rampController;

    ///

    ///
    protected IntakeController intakeController;

    ///


    public CalibrateSpindexerSubTree(LinearOpMode opMode, Telemetry telemetry) {
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

        /// Switch
        this.switchController = SpindexerLimitSwitchController.getInstance();
        /// End Switch

        /// Ramp
        this.rampController = RampController.getInstance();
        /// End Ramp

        /// Intake
        this.intakeController = IntakeController.getInstance();
        /// End Intake


        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new StoreRamp(telemetry, rampController), telemetry),
                        new Action(new PauseAction(telemetry, 500), telemetry),
                        new Action(new RetainArtifacts(telemetry, intakeController), telemetry),
                        new Action(new ReadSwitch(telemetry, switchController), telemetry),
                        new Action(new RunSpindexerToReference(telemetry, spindexerController), telemetry),
                        new Action(new StopIntake(telemetry, intakeController), telemetry)
                ), telemetry);

        this.tree = new BehaviorTree(root, blackBoard);
    }

    public Node getRoot() {
        // *****DON'T CLEAR THE HUBS HERE BECAUSE THIS IS A SUBTREE*****

        // Return the array of actions as a sub-tree
        return root;
    }
}

