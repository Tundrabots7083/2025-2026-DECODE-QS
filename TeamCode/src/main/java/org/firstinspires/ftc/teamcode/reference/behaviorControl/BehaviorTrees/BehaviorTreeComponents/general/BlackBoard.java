package org.firstinspires.ftc.teamcode.reference.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general;

/*import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageVisionDetectorSingleton;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;*/

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class BlackBoard {
    private static BlackBoard instance;
    private final Map<String, Object> data = new HashMap<String, Object>();

    private Telemetry telemetry=null;
    // Private constructor to prevent instantiation from other classes
    private BlackBoard(Telemetry telemetry) {
        this.telemetry = telemetry;

    }

    // Public static method to get the single instance of the class
    public static BlackBoard getInstance(Telemetry telemetry) {
        if (instance == null) {
            instance = new BlackBoard(telemetry);
        }
        return instance;
    }

    public void reset(){
        instance = null;
    }

    public void setValue(String key, Object value) {
        data.put(key, value);
    }
    public Object removeValue(String key) {
        return data.remove(key);
    }

    public Object getValue(String key) {
        return data.get(key);
    }

}
