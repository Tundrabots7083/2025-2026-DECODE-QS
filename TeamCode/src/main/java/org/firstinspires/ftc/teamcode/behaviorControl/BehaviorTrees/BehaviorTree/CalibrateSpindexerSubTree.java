package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.LimitSwitch.ReadSwitch;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.RunSpindexerToReference;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Spindexer.SpinToZeroPosition;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer.SpindexerController;
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

        this.spindexerController.reset();
        this.spindexerController.initialize(hardwareMap, telemetry);
        ///  End Spindexer

        /// Switch
        this.switchController = SpindexerLimitSwitchController.getInstance();

        this.switchController.reset();
        this.switchController.initialize(hardwareMap, telemetry);
        /// End Switch


        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new ReadSwitch(telemetry, switchController), telemetry),
                        new Action(new RunSpindexerToReference(telemetry, spindexerController), telemetry),
                        new Action(new SpinToZeroPosition(telemetry, spindexerController), telemetry)
                ), telemetry);

        this.tree = new BehaviorTree(root, blackBoard);
    }

    public Node getRoot() {
        // *****DON'T CLEAR THE HUBS HERE BECAUSE THIS IS A SUBTREE*****

        // Run the behavior tree
        return root;
    }
}

