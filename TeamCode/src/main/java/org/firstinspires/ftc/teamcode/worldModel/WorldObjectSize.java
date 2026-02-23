package org.firstinspires.ftc.teamcode.worldModel;

import androidx.annotation.NonNull;

public class WorldObjectSize {
    public double width;
    public double height;

    public WorldObjectSize(double width, double height) {
        this.width = width;
        this.height = height;

    }

    @NonNull
    @Override
    public String toString() {
        return " WorldObjectSize [width = " + width + ", height = " + height + "]";
    }
}