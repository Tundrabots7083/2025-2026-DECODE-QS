package org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.LimeLightConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.limeLight.LimeLight01Constants;

import java.util.List;

public class LimeLightController  {
    private boolean initialized = false;


    // Private static instance (eager initialization)
    private static final LimeLightController INSTANCE = new LimeLightController();
    private LinearOpMode opMode;
    private Telemetry telemetry;

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
        limelight = hardwareMap.get(Limelight3A.class, LimeLightConstants.name);
        limelight.pipelineSwitch(0); //TODO: match the pipeline index to LL config
        limelight.start();
    }

    // Example method
    public Pose3D getCurrentRobotPose(){
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            Pose3D robotPose = result.getBotpose();

            telemetry.addData("RobotPose", robotPose.toString());
            return robotPose;
        }
        return null;
    }

public   List<LLResultTypes.FiducialResult> getFiducialResults(){
    LLStatus status = limelight.getStatus();
    telemetry.addData("Name", "%s",
            status.getName());
    telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
            status.getTemp(), status.getCpu(),(int)status.getFps());
    telemetry.addData("Pipeline", "Index: %d, Type: %s",
            status.getPipelineIndex(), status.getPipelineType());

    LLResult result = limelight.getLatestResult();
    if (result.isValid()) {
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
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

