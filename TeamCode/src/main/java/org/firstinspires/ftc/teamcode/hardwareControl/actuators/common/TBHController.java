package org.firstinspires.ftc.teamcode.hardwareControl.actuators.common;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TBHController {
    private double Kp, Kf_a, Kf_b, Kf_c; // error multiplier coefficients
    private double lastError = 0.0;
    private double maxPower = 1.0;
    private double minPower = -1.0;
    private boolean isFirstCross = true;
    private double driveAtZero = 0.0; // This is the previous motor power at zero error
    private double power = 0.0;
    private Telemetry telemetry;

    /**
     * Constructor for TBHController
     *
     * @param Kp Proportional gain
     */
    public TBHController(double Kp, double Kf_a, double Kf_b, double Kf_c, Telemetry telemetry) {
        this.Kp = Kp;
        this.Kf_a = Kf_a;
        this.Kf_b = Kf_b;
        this.Kf_c = Kf_c;
        this.telemetry = telemetry;
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
        telemetry.addData("Error", error);

        if (Math.abs(error) >= 400) {
            isFirstCross = true;
            lastError = error;
            return Math.signum(error);
        } else if(isFirstCross && driveAtZero == 0.0) {
            power = Kf_a
                    + Kf_b * setpoint
                    + Kf_c * Math.pow(setpoint, 2);
            driveAtZero = power;
            isFirstCross = false;
            lastError = error;
            return power;
        } else if (isFirstCross) {
            isFirstCross = false;
            return driveAtZero;
        }


        // Increment Power
        power += (Kp * error);

        // Clamp output to motor power limits
        power = Range.clip(power,minPower,maxPower);

        telemetry.addData("Power in TBH", power);

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
    public void setP(double Kp) {
        this.Kp = Kp;
    }

    public double getKp() { return Kp; }
}