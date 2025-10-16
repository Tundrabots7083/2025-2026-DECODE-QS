package org.firstinspires.ftc.teamcode.hardwareControl.actuators.common;

public class TBHController {
    private double kP, kF; // error multiplier coefficients
    private double lastError = 0.0;
    private double maxPower = 1.0; // Typical max power for FTC motors
    private double minPower = 0.0; // Only going forwards in velocity control
    private boolean isFirstCross = true;
    private double driveAtZero = 0.0; // This is the previous motor power at zero error
    private double power = 0.0;

    /**
     * Constructor for TBHController
     * @param kP Proportional gain
     */
    public TBHController(double kP, double kF) {
        this.kP = kP;
        this.kF = kF;
    }

    /**
     * Sets the output limits for the controller
     * @param min Minimum output (e.g., -1.0 for motor power)
     * @param max Maximum output (e.g., 1.0 for motor power)
     */
    public void setOutputLimits(double min, double max) {
        this.minPower = min;
        this.maxPower = max;
    }

    /**
     * Calculates the control output based on setpoint and current position
     * @param setpoint Desired target (e.g., encoder ticks or velocity)
     * @param current Current position or state (e.g., encoder ticks or velocity)
     * @return Control output (e.g., motor power between -1.0 and 1.0)
     */
    public double calculate(double setpoint, double current) {

        // Calculate error
        double error = setpoint - current;

        // Increment Power
        power = power + (kP * error);

        // Clamp output to motor power limits
        power = Math.max(Math.min(power, maxPower), minPower);

        // Doesn't run if this is the first loop, otherwise check if crossed target
        if ((lastError != 0.0) && (Math.signum(error) != Math.signum(lastError))) {
            if (isFirstCross) {
                power = kF * setpoint;
                isFirstCross = false;
            } else {
                // Average previous best guess with latest best guess
                power = 0.5 * (power + driveAtZero);
            }

            // Store the latest best guess at the correct motor power
            driveAtZero = power;

        }

        lastError = error;

        return power;
    }

    /**
     * Resets the controller's integral and last error
     */
    public void reset() {
        lastError = 0.0;
        isFirstCross = true;
    }

    /**
     * Getters and setters for PIDF coefficients
     */
    public void setPIDF(double kP, double kF) {
        this.kP = kP;
        this.kF = kF;
    }

    public double getKP() { return kP; }
    public double getkF() { return kF; }
}