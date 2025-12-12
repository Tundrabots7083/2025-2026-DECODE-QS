package org.firstinspires.ftc.teamcode.hardwareControl.actuators.shooter;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.shooter.ShooterTBHControllerConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.MotorConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.PIDFControllerConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ShooterConstantsBase;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.TBHController;

@Configurable
public class ShooterController {

     private DcMotorEx shooterMotor;

    private double START_VELOCITY;
    private double TOLERABLE_ERROR;
    private double lastPower = 0;
    private double targetVelocity;

    public static double Kp = 0.02;
    public static double Kf = 0.0121;


    /// motion control
    private TBHController tbhController;

    private double currentVelocity;
    ///

    private boolean initialized = false;

    // Private static instance (eager initialization)
    private static final ShooterController INSTANCE = new ShooterController();
    private Telemetry telemetry;

    // Private constructor to prevent instantiation
    private ShooterController() {

    }

    // Public method to access the singleton instance
    public static ShooterController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants(){
        try {
            Class.forName(ShooterConstants.class.getName());
            Class.forName(ShooterTBHControllerConstants.class.getName());
        } catch (ClassNotFoundException e) {
            //e.printStackTrace();
        }
    }
    // Initialization method — must be called once at the beginning
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        if (initialized) {
            return;
            //throw new IllegalStateException("ShoulderController has already been initialized.");
        }
        setupConstants();
        this.telemetry  = telemetry;

        initializeMotor(hardwareMap);
        initializeLocalVariablesWithConstants();
        initializeTBHController();

        initialized = true;
    }

    private void initializeMotor(HardwareMap hardwareMap){
        shooterMotor =  hardwareMap.get(DcMotorEx.class, ShooterConstantsBase.frontMotorName);
        MotorConfigurationType motorConfigurationType = shooterMotor.getMotorType().clone();
        motorConfigurationType.setTicksPerRev(MotorConstants.ticksPerRev);
        motorConfigurationType.setGearing(MotorConstants.gearing);
        motorConfigurationType.setAchieveableMaxRPMFraction(MotorConstants.achievableMaxRPMFraction);
        shooterMotor.setMotorType(motorConfigurationType);
        shooterMotor.setMode(MotorConstants.resetMode);
        shooterMotor.setMode(MotorConstants.mode);
        shooterMotor.setDirection(MotorConstants.direction);
    }

    private void initializeLocalVariablesWithConstants(){
        START_VELOCITY = MotorConstants.startPosition;
        TOLERABLE_ERROR = MotorConstants.tolerableError;
    }

    private void initializeTBHController(){

        tbhController = new TBHController(Kp, Kf);
        tbhController.setOutputLimits(PIDFControllerConstants.motorMinPowerLimit, PIDFControllerConstants.motorMaxPowerLimit); // Motor power limits
    }


    public void reset() {
        /// TODO: include sensor to detect hardware reset.
        if(initialized) {
            if(this.isBusy()) {
                this.spinToTargetVelocity(START_VELOCITY);
            } else {
                shooterMotor.setMode(MotorConstants.resetMode);
                shooterMotor.setMode(MotorConstants.mode);
                shooterMotor.setDirection(MotorConstants.direction);
            }
            initialized = false;
        }
    }

    public double getCurrentPosition() {
        return shooterMotor.getCurrentPosition();
    }


    /**
     * Gets the velocity of the shooter motor
     * @return returns rotations per minute
     */
    public double getCurrentVelocity() {
        return -shooterMotor.getVelocity(AngleUnit.DEGREES) / 6;
    }

    public void spinToTargetVelocity(double newTargetVelocity){

        if(newTargetVelocity != targetVelocity) {
            tbhController.reset();
        }

        targetVelocity = newTargetVelocity;
        currentVelocity = getCurrentVelocity();

        // Calculate motor tbhPower using TBH
        double tbhPower = tbhController.calculate(targetVelocity, currentVelocity);

        //Apply power to the motor if this is the first loop
        //Or if it's substantially different than what the motor is currently running at.
        //Otherwise save time by ignoring small changes in power.
        if ((lastPower == 0.0) || (Math.abs(tbhPower - lastPower) >= 0.005)) {
            // Apply tbhPower to motor
            shooterMotor.setPower(tbhPower);
            telemetry.addData("Set Power to PID", tbhPower);
        }

    }

    public boolean isOnTarget(){
          double currentVelocity = this.getCurrentVelocity();

        return ((Math.abs(targetVelocity - currentVelocity) <= TOLERABLE_ERROR));
    }

    public void update(){

    }

    public boolean isBusy(){
        return shooterMotor.isBusy();
    }
}
