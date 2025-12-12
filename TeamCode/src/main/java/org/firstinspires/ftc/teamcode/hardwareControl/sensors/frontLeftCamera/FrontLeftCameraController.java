package org.firstinspires.ftc.teamcode.hardwareControl.sensors.frontLeftCamera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.WebCamConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.frontLeftCamera.FrontLeftCameraConstants;


public class FrontLeftCameraController {
    private boolean initialized = false;


    // Private static instance (eager initialization)
    private static final FrontLeftCameraController INSTANCE = new FrontLeftCameraController();
    private LinearOpMode opMode;
    private Telemetry telemetry;

    private DistanceSensor distanceSensor;
    private DistanceUnit distanceUnit;

    private WebcamName webCamFL;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Private constructor to prevent instantiation
    private FrontLeftCameraController() {
        // Initialize hardware, state, or configuration here

    }

    // Public method to access the singleton instance
    public static FrontLeftCameraController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants(){
        try {
            Class.forName(FrontLeftCameraConstants.class.getName());
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


        initAprilTag(hardwareMap);


        initialized = true;
    }

    public void reset() {

    }

    private void initAprilTag(HardwareMap hardwareMap) {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the WEBCAM vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WebCamConstants.name))
                .addProcessor(aprilTag)
                .build();
    }

    // Example method
    public void getCurrentDetections(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                telemetry.addData("FLCamCtrl", "\n==== (ID %d) %s", detection.id, detection.metadata.name);
                telemetry.addData("FLCamCtrl", "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                telemetry.addData("FLCamCtrl", "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);
                telemetry.addData("FLCamCtrl", "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);
            } else {
                telemetry.addData("FLCamCtrl", "\n==== (ID %d) Unknown", detection.id);
                telemetry.addData("FLCamCtrl", "Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y);
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }



    public void update(){

    }

    public void resumeStreaming(){
        visionPortal.resumeStreaming();
    }

    public void stopStreaming(){
        visionPortal.stopStreaming();
    }

}

/*usage Example*/


