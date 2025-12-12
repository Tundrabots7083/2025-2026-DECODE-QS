package org.firstinspires.ftc.teamcode.hardwareControl.actuators.common;

public class MotionProfiler {
    private double maxVelocity; // Maximum velocity (e.g., ticks per second)
    private double maxAcceleration; // Maximum acceleration (e.g., ticks per second^2)
    private double targetPosition; // Target position (e.g., encoder ticks)
    private double initialPosition; // Starting position (e.g., encoder ticks)
    private double accelTime; // Time to reach max velocity
    private double cruiseTime; // Time at max velocity
    private double totalTime; // Total time of motion
    private boolean isProfileGenerated = false;

    /**
     * Constructor for MotionProfiler
     * @param maxVelocity Maximum velocity (e.g., ticks per second)
     * @param maxAcceleration Maximum acceleration (e.g., ticks per second^2)
     */
    public MotionProfiler(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    /**
     * Generates a trapezoidal motion profile
     * @param initialPosition Starting position (e.g., encoder ticks)
     * @param targetPosition Desired final position (e.g., encoder ticks)
     */
    public void generateProfile(double initialPosition, double targetPosition) {
        this.initialPosition = initialPosition;
        this.targetPosition = targetPosition;

        // Calculate distance to travel
        double distance = Math.abs(targetPosition - initialPosition);

        // Calculate time to accelerate/decelerate
        accelTime = maxVelocity / maxAcceleration;

        // Calculate distance covered during acceleration and deceleration
        double accelDistance = 0.5 * maxAcceleration * accelTime * accelTime;

        // Check if we can reach max velocity (triangular vs. trapezoidal profile)
        if (accelDistance * 2 > distance) {
            // Triangular profile (no cruise phase)
            accelTime = Math.sqrt(distance / maxAcceleration);
            maxVelocity = maxAcceleration * accelTime;
            cruiseTime = 0;
        } else {
            // Trapezoidal profile
            double cruiseDistance = distance - 2 * accelDistance;
            cruiseTime = cruiseDistance / maxVelocity;
        }

        totalTime = 2 * accelTime + cruiseTime;
        isProfileGenerated = true;
    }

    /**
     * Gets the desired position and velocity at a given time
     * @param time Current time since profile start (seconds)
     * @return MotionState containing position and velocity
     */
    public MotionState getMotionState(double time) {
        if (!isProfileGenerated) {
            return new MotionState(initialPosition, 0);
        }

        double direction = Math.signum(targetPosition - initialPosition);
        double position = initialPosition;
        double velocity = 0;

        if (time < 0) {
            return new MotionState(initialPosition, 0);
        } else if (time < accelTime) {
            // Acceleration phase
            velocity = maxAcceleration * time;
            position = initialPosition + direction * 0.5 * maxAcceleration * time * time;
        } else if (time < accelTime + cruiseTime) {
            // Cruise phase
            velocity = maxVelocity;
            position = initialPosition + direction * (0.5 * maxAcceleration * accelTime * accelTime +
                    maxVelocity * (time - accelTime));
        } else if (time < totalTime) {
            // Deceleration phase
            double t = time - (accelTime + cruiseTime);
            velocity = maxVelocity - maxAcceleration * t;
            position = initialPosition + direction * (0.5 * maxAcceleration * accelTime * accelTime +
                    maxVelocity * cruiseTime +
                    maxVelocity * t - 0.5 * maxAcceleration * t * t);
        } else {
            // Motion complete
            velocity = 0;
            position = targetPosition;
        }

        return new MotionState(position, velocity * direction);
    }

    /**
     * Returns the total time of the motion profile
     * @return Total time in seconds
     */
    public double getTotalTime() {
        return totalTime;
    }

    /**
     * Inner class to hold position and velocity
     */
    public static class MotionState {
        public final double position;
        public final double velocity;

        public MotionState(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }
    }
}