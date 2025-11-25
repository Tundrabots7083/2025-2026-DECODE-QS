package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.sensors.limeLight.DisplayPose;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.sensors.limeLight.RetrieveAprilTagPose;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight.LimeLightController;

import java.util.Arrays;
import java.util.List;


public class DetectPoseBehaviorTree {
    private BehaviorTree tree;
    private Node root;
    private BlackBoard blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected LinearOpMode opMode;


    /// Limelight
    protected LimeLightController limeLightController;

    /// End Limelight

    public DetectPoseBehaviorTree(LinearOpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

        Init();
    }
    private void Init() {
        this.blackBoard = BlackBoard.getInstance(telemetry);
        this.blackBoard.reset();


        /// Limelight
        this.limeLightController = LimeLightController.getInstance();
        this.limeLightController.initialize(hardwareMap, telemetry);

        /// End Limelight


        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new RetrieveAprilTagPose(telemetry, limeLightController), telemetry),
                        new Action(new DisplayPose(telemetry), telemetry)
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

        telemetry.update();

        // Run the behavior tree
        Status result = tree.tick();

        return result;
    }
}

