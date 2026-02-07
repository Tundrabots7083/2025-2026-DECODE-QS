package org.firstinspires.ftc.teamcode.worldModel;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DecodeWorldModel {
    private static DecodeWorldModel instance;
    private final Map<String, WorldObject> worldModelObjects = new HashMap<>();

    private Telemetry telemetry=null;
    // Private constructor to prevent instantiation from other classes
    private DecodeWorldModel(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.init();

    }

    public DecodeWorldModel() {
    }

    // Public static method to get the single instance of the class
    public static DecodeWorldModel getInstance(Telemetry telemetry) {
        if (instance == null) {
            instance = new DecodeWorldModel(telemetry);
        }
        return instance;
    }

    public void reset(){
        instance = null;
    }

    public void setValue(WorldObject value) {
        worldModelObjects.put(value.key, value);
    }

    public WorldObject getValue(String key) {
        return worldModelObjects.get(key);
    }

    private void init(){
        /// RED ALLIANCE ////////////////////////////////
        /// Red Alliance Goal
        WorldObject redAllianceGoal = new WorldObject("RedAllianceGoal", "RedAllianceGoal", new Position(DistanceUnit.INCH, 144, 144, 48.0, 10), null);

        /// Red Alliance Spikes
        WorldObject redAllianceAudienceSpike = new WorldObject("RedAllianceAudienceSpike","RedAllianceAudienceSpike",  new Position(DistanceUnit.INCH,120.0, 36,0.0,10), null);
        WorldObject redAllianceMiddleSpike = new WorldObject("RedAllianceMiddleSpike","RedAllianceMiddleSpike", new Position(DistanceUnit.INCH,120.0, 60.0,0.0,10), null);
        WorldObject redAllianceFrontSpike = new WorldObject("RedAllianceFrontSpike","RedAllianceFrontSpike", new Position(DistanceUnit.INCH,120.0, 84.0,0.0,10), null);

        /// Red Alliance Gate
        WorldObject redAllianceGate = new WorldObject("RedAllianceGate","RedAllianceGate", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));

        this.setValue(redAllianceGoal);

        this.setValue(redAllianceAudienceSpike);
        this.setValue(redAllianceMiddleSpike);
        this.setValue(redAllianceFrontSpike);

        this.setValue(redAllianceGate);


        ///  END RED ALLIANCE ////////////////////////////


        /// BLUE ALLIANCE ////////////////////////////////
        /// Blue Alliance Goal
        WorldObject blueAllianceGoal = new WorldObject("BlueAllianceGoal", "BlueAllianceGoal", new Position(DistanceUnit.INCH, 9.0, 135, 48, 10), null);

        /// Blue Alliance Spikes
        WorldObject blueAllianceAudienceSpike = new WorldObject("BlueAllianceAudienceSpike","BlueAllianceAudienceSpike", new Position(DistanceUnit.INCH,24.0, 46.0,0.0,10), null);
        WorldObject blueAllianceMiddleSpike = new WorldObject("BlueAllianceMiddleSpike","BlueAllianceMiddleSpike", new Position(DistanceUnit.INCH,24.0, 60.0,0.0,10), null);
        WorldObject blueAllianceFrontSpike = new WorldObject("BlueAllianceFrontSpike","BlueAllianceFrontSpike", new Position(DistanceUnit.INCH,24.0, 84.0,0.0,10), null);

        /// Blue Alliance Gate
        WorldObject blueAllianceGate = new WorldObject("BlueAllianceGate","BlueAllianceGate", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));

        this.setValue(blueAllianceGoal);

        this.setValue(blueAllianceAudienceSpike);
        this.setValue(blueAllianceMiddleSpike);
        this.setValue(blueAllianceFrontSpike);

        this.setValue(blueAllianceGate);


        ///  END BLUE ALLIANCE ////////////////////////////
    }
    public /* WorldObject[] */ void getValues(List<String> keys) {
/*
        var x = worldModelObjects.entrySet()
                .stream()
                .filter((entry) -> keys.contains(entry.getKey()))
                .map(Map.Entry::getValue)
                .collect(Collectors.toList());

        System.out.println("World model values" + x);
        */
        // return
        // worldModelObjects.entrySet().stream().filter((entry)->keys.equals(entry.getKey()));
    }
}

