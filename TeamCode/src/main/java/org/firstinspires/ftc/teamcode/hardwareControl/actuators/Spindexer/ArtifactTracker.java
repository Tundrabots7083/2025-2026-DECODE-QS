package org.firstinspires.ftc.teamcode.hardwareControl.actuators.Spindexer;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareControl.sensors.storageInventoryController.ArtifactColor;

@Configurable
public class ArtifactTracker {

    private static final int PATTERN_SIZE = 3;

    // Singleton instance
    private static final ArtifactTracker INSTANCE = new ArtifactTracker();

    private boolean initialized = false;
    private ArtifactColor[] currentPattern;
    private Telemetry telemetry;

    private ArtifactTracker() {
        // prevent external construction
    }

    public static ArtifactTracker getInstance() {
        return INSTANCE;
    }

    /**
     * Must be called once per OpMode before use
     */
    public void initialize(Telemetry telemetry) {
        if (initialized) return;

        this.telemetry = telemetry;
        currentPattern = new ArtifactColor[PATTERN_SIZE];
        clearPattern();
        telemetry.addLine("HasBeenInitialized");
        initialized = true;
    }

    /**
     * Clears stored artifacts but keeps tracker usable
     */
    public void clearPattern() {
        for (int i = 0; i < PATTERN_SIZE; i++) {
            currentPattern[i] = ArtifactColor.NONE;
        }
    }

    /**
     * Sets artifact color at a given index
     */
    public void setArtifact(int position, ArtifactColor artifactColor) {

        if (position < 0 || position >= PATTERN_SIZE) {
            throw new IllegalArgumentException(
                    "Artifact position out of range: " + position
            );
        }

        currentPattern[position] = artifactColor;
    }

    /**
     * Safe read-only access
     */
    public ArtifactColor getArtifact(int position) {

        if (position < 0 || position >= PATTERN_SIZE) {
            throw new IllegalArgumentException(
                    "Artifact position out of range: " + position
            );
        }

        return currentPattern[position];
    }

    /**
     * Returns a defensive copy to prevent external mutation
     */
    public ArtifactColor[] getCurrentPatternSnapshot() {
        return currentPattern;
    }

    public void setCurrentPatternSnapshot(ArtifactColor[] newPattern) {
        currentPattern = newPattern;
    }

    /** Explicit lifecycle reset */
    public void reset() {
        initialized = false;
        currentPattern = null;
        telemetry = null;
    }

}
