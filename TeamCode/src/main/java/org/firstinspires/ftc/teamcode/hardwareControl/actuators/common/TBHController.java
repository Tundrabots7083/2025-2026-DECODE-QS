package org.firstinspires.ftc.teamcode.hardwareControl.actuators.common;

import com.qualcomm.robotcore.util.Range;

public class TBHController {
    private double kP, kF_a, kF_b, kF_c; // error multiplier coefficients
    private double lastError = 0.0;
    private double maxPower = 1.0; // Typical max power for FTC motors
    private double minPower = -1.0;
    private boolean isFirstCross = true;
    private double driveAtZero = 0.0; // This is the previous motor power at zero error
    private double power = 0.0;

    /**
     * Constructor for TBHController
     *
     * @param kP Proportional gain
     */
    public TBHController(double kP, double kF_a, double kF_b, double kF_c) {
        this.kP = kP;
        this.kF_a = kF_a;
        this.kF_b = kF_b;
        this.kF_c = kF_c;
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

        if (Math.abs(error) > 400) {
            isFirstCross = true;
            lastError = error;
            return Math.signum(error);
        } else if(isFirstCross && driveAtZero == 0.0) {
            power = kF_a
                    + kF_b * setpoint
                    + kF_c * Math.pow(setpoint, 2);
            driveAtZero = power;
            isFirstCross = false;
            lastError = error;
            return power;
        } else if (isFirstCross) {
            isFirstCross = false;
            return driveAtZero;
        }


        // Increment Power
        power = power + (kP * error);

        // Clamp output to motor power limits
        power = Range.clip(power, minPower, maxPower);

        // Doesn't run if this is the first loop, otherwise check if crossed target
        if ((lastError != 0.0) && (Math.signum(error) != Math.signum(lastError))) {
                // Average previous best guess with latest best guess
                power = 0.5 * (power + driveAtZero);

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
        driveAtZero = 0.0;
        power = 0.0;
        isFirstCross = true;
    }

    /**
     * Getters and setters for PIDF coefficients
     */
    public void setPIDF(double kP) {
        this.kP = kP;
    }

    public double getKP() { return kP; }
}