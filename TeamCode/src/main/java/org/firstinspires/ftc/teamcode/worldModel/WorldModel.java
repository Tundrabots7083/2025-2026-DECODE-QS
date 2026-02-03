package org.firstinspires.ftc.teamcode.worldModel;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class WorldModel {
    private static WorldModel instance;
    private final Map<String, WorldObject> worldModelObjects = new HashMap<>();

    private Telemetry telemetry=null;
    // Private constructor to prevent instantiation from other classes
    private WorldModel(Telemetry telemetry) {
        this.telemetry = telemetry;

    }

    public WorldModel() {
    }

    // Public static method to get the single instance of the class
    public static WorldModel getInstance(Telemetry telemetry) {
        if (instance == null) {
            instance = new WorldModel(telemetry);
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

