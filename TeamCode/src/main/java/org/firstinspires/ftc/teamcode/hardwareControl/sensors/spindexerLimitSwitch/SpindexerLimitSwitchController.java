package org.firstinspires.ftc.teamcode.hardwareControl.sensors.spindexerLimitSwitch;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class SpindexerLimitSwitchController {

    private DigitalChannel limitSwitch;

    private boolean initialized = false;

    // Singleton instance
    private static final SpindexerLimitSwitchController INSTANCE = new SpindexerLimitSwitchController();
    private Telemetry telemetry;

    private SpindexerLimitSwitchController() {
    }

    public static SpindexerLimitSwitchController getInstance() {
        return INSTANCE;
    }


    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) return;

        this.telemetry = telemetry;

        initializeLimitSwitch(hardwareMap);

        initialized = true;
    }

    private void initializeLimitSwitch(HardwareMap hardwareMap) {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "Spindexer Switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    // button is pressed if value returned is LOW or false.
    public boolean getState() {
        return limitSwitch.getState();
    }

    public void reset() {
        if (!initialized) return;
        initialized = false;
    }
}