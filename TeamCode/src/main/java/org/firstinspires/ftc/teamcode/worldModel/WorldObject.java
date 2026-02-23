package org.firstinspires.ftc.teamcode.worldModel;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class WorldObject {
    public String name;
    public String key;
    public Position position;
    public Pose2D pose2D;
    public Pose3D pose3D;
    @Nullable
    public WorldObjectSize size;

    public WorldObject(String name, String key, Position position, @Nullable WorldObjectSize size) {
        this.name = name;
        this.key = key;
        this.position = position;
        this.size = size;
    }
    public WorldObject(String name, String key, Pose2D pose2D, @Nullable WorldObjectSize size) {
        this.name = name;
        this.key = key;
        this.pose2D = pose2D;
        this.size = size;
    }
    public WorldObject(String name, String key, Pose3D pose3D, @Nullable WorldObjectSize size) {
        this.name = name;
        this.key = key;
        this.pose3D = pose3D;
        this.size = size;
    }
    @NonNull
    @Override
    public String toString() {
        return "\nWorldObject [key = " + key + ", position =" + pose2D + ", size  =" + size + "]";
    }
}

