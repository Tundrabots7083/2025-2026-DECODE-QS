package org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor;

@Configurable
public class ArtifactTracker {
    private boolean initialized = false;

    private ArtifactColor[] currentPattern;

    // Singleton instance
    private static final ArtifactTracker INSTANCE = new ArtifactTracker();
    private Telemetry telemetry;

    private ArtifactTracker() {
    }

    public static ArtifactTracker getInstance() {
        return INSTANCE;
    }


    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) return;

        currentPattern = new ArtifactColor[]{ArtifactColor.NONE, ArtifactColor.NONE, ArtifactColor.NONE};
        this.telemetry = telemetry;
        initialized = true;
    }

    public void addArtifact(int position, ArtifactColor artifactColor) {
        currentPattern[position] = artifactColor;
    }

    public ArtifactColor[] getCurrentPattern() {
        return currentPattern;
    }

    public void update() {
    }

    public void reset() {
        if (!initialized) return;

        for (int i = 0; i < 2; i++) {
            currentPattern[i] = ArtifactColor.NONE;
        }
        initialized = false;
    }
}
