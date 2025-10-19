package org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.limeLight.LimeLight01Constants;

import java.util.List;

public class LimeLightController  {
    private boolean initialized = false;


    // Private static instance (eager initialization)
    private static final LimeLightController INSTANCE = new LimeLightController();
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private final int APRILTAG_PIPELINE = 0;

    private Limelight3A limelight;


    // Private constructor to prevent instantiation
    private LimeLightController() {
        // Initialize hardware, state, or configuration here

    }

    // Public method to access the singleton instance
    public static LimeLightController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants(){
        try {
            Class.forName(LimeLight01Constants.class.getName());
        } catch (ClassNotFoundException e) {
            //e.printStackTrace();
        }
    }
    // Initialization method — must be called once at the beginning
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        if (initialized) {
            return;
            //throw new IllegalStateException("FrontLeftCameraController has already been initialized.");
        }
        setupConstants();
        this.opMode = opMode;
        this.telemetry  = telemetry;


        initLimeLight01(hardwareMap);


        initialized = true;
    }

    public void reset() {

    }

    private void initLimeLight01(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    // Example method
    public Pose3D getCurrentRobotPose(){
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            Pose3D robotPose = result.getBotpose();

            double xPose = (robotPose.getPosition().x * 39.37008);  //convert meters to inches
            double yPose = (robotPose.getPosition().y * 39.37008);  //convert meters to inches
            double heading = robotPose.getOrientation().getYaw();

            telemetry.addData("RobotPoseX", xPose);
            telemetry.addData("RobotPoseY", yPose);
            telemetry.update();
            return robotPose;
        }
        return null;
    }

public   List<LLResultTypes.FiducialResult> getFiducialResults(){
    LLResult result = limelight.getLatestResult();
    if (result.isValid()) {
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
        }
        return fiducialResults;
    }
    return null;
}

    public void update(){

    }

    public void resumeStreaming(){
        limelight.start();
    }

    public void stopStreaming(){
        limelight.stop();
    }

}

/*usage Example*/

