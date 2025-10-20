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

    private DcMotorEx frontShooterMotor;
    private DcMotorEx rearShooterMotor;

    private double START_VELOCITY;
    private double TOLERABLE_ERROR;
    private double lastPower = 0;
    private double targetVelocity;

//    double FRONT_Kp = ShooterTBHControllerConstantBase.FRONT_Kp;
//    double FRONT_Kf = ShooterTBHControllerConstantBase.FRONT_Kf;
//    double REAR_Kp = ShooterTBHControllerConstantBase.FRONT_Kp;
//    double REAR_Kf = ShooterTBHControllerConstantBase.FRONT_Kf;

    public static double FRONT_Kp =0.000002;
    public static double FRONT_Kf =0.00017;
    public static double REAR_Kp = 0.000002;
    public static double REAR_Kf = 0.00017;


    /// motion control
    private TBHController frontTbhController;
    private TBHController rearTbhController;

    private double currentFrontVelocity;
    private double currentRearVelocity;

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
        frontShooterMotor =  hardwareMap.get(DcMotorEx.class, ShooterConstantsBase.frontMotorName);
        MotorConfigurationType motorConfigurationType = frontShooterMotor.getMotorType().clone();
        rearShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstantsBase.rearMotorName);
        motorConfigurationType.setTicksPerRev(MotorConstants.ticksPerRev);
        motorConfigurationType.setGearing(MotorConstants.gearing);
        motorConfigurationType.setAchieveableMaxRPMFraction(MotorConstants.achievableMaxRPMFraction);

        frontShooterMotor.setMotorType(motorConfigurationType);
        frontShooterMotor.setMode(ShooterConstantsBase.resetMode);
        frontShooterMotor.setMode(ShooterConstantsBase.mode);
        frontShooterMotor.setDirection(ShooterConstantsBase.frontMotorDirection);

        rearShooterMotor.setMotorType(motorConfigurationType);
        rearShooterMotor.setMode(ShooterConstantsBase.resetMode);
        rearShooterMotor.setMode(ShooterConstantsBase.mode);
        rearShooterMotor.setDirection(ShooterConstantsBase.rearMotorDirection);
    }

    private void initializeLocalVariablesWithConstants(){
        START_VELOCITY = MotorConstants.startPosition;
        TOLERABLE_ERROR = MotorConstants.tolerableError;
    }

    private void initializeTBHController(){

        frontTbhController = new TBHController(FRONT_Kp, FRONT_Kf);
        frontTbhController.setOutputLimits(PIDFControllerConstants.motorMinPowerLimit, PIDFControllerConstants.motorMaxPowerLimit); // Motor power limits

        rearTbhController = new TBHController(REAR_Kp, REAR_Kf);
        rearTbhController.setOutputLimits(PIDFControllerConstants.motorMinPowerLimit, PIDFControllerConstants.motorMaxPowerLimit); // Motor power limits
    }


    public void reset() {
        /// TODO: include sensor to detect hardware reset.
        if(initialized) {
            if(this.isBusy()) {
                this.spinToTargetVelocity(START_VELOCITY);
            } else {
                frontShooterMotor.setMode(MotorConstants.resetMode);
                frontShooterMotor.setMode(MotorConstants.mode);
                frontShooterMotor.setDirection(MotorConstants.direction);

                rearShooterMotor.setMode(MotorConstants.resetMode);
                rearShooterMotor.setMode(MotorConstants.mode);
                rearShooterMotor.setDirection(MotorConstants.direction);
            }
            initialized = false;
        }
    }

    public double getFrontCurrentPosition() {
        return frontShooterMotor.getCurrentPosition();
    }

    public double getRearCurrentPosition() {
        return rearShooterMotor.getCurrentPosition();
    }


    /**
     * Gets the velocity of the shooter motor
     * @return returns rotations per minute
     */
    public double getFrontCurrentVelocity() {
        return frontShooterMotor.getVelocity(AngleUnit.DEGREES) / 6;
    }

    /**
     * Gets the velocity of the shooter motor
     * @return returns rotations per minute
     */
    public double getRearCurrentVelocity() {
        return rearShooterMotor.getVelocity(AngleUnit.DEGREES) / 6;
    }

    public void spinToTargetVelocity(double newTargetVelocity){

        if(newTargetVelocity != targetVelocity) {
            frontTbhController.reset();
            rearTbhController.reset();
        }

        targetVelocity = newTargetVelocity;

        currentFrontVelocity = getFrontCurrentVelocity();
        currentRearVelocity = getRearCurrentVelocity();

        // Calculate motor tbhPower using TBH
        double FRONTtbhPower = frontTbhController.calculate(targetVelocity, currentFrontVelocity);
        double REARtbhPower = rearTbhController.calculate(targetVelocity, currentRearVelocity);


        //Apply power to the motor if this is the first loop
        //Or if it's substantially different than what the motor is currently running at.
        //Otherwise save time by ignoring small changes in power.
//        if ((lastPower == 0.0) || (Math.abs(REARtbhPower - lastPower) >= 0.005)) {
            // Apply FRONTtbhPower to motor
            frontShooterMotor.setPower(FRONTtbhPower);
            rearShooterMotor.setPower(REARtbhPower);
            telemetry.addData("Set Front Power to PID", FRONTtbhPower);
            telemetry.addData("Set Rear Power to PID", REARtbhPower);
//        }

    }

    public boolean isOnTarget(){
          double currentFrontVelocity = getFrontCurrentVelocity();
          double currentRearVelocity = getRearCurrentVelocity();

        return ((Math.abs(targetVelocity - currentFrontVelocity) <= TOLERABLE_ERROR) && (Math.abs(targetVelocity - currentRearVelocity) <= TOLERABLE_ERROR));
    }

    public void update(){

    }

    public boolean isBusy(){
        return frontShooterMotor.isBusy() || rearShooterMotor.isBusy();
    }
}
