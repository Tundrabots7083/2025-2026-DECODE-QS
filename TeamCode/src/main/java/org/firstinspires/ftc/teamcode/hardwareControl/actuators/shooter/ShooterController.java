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
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.VelocityMotorConstantsBase;
import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.VelocityTBHControllerConstantBase;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.TBHController;

@Configurable
public class ShooterController {

    private DcMotorEx frontShooterMotor;
    private DcMotorEx rearShooterMotor;

    private double START_VELOCITY;
    private double TOLERABLE_ERROR;
    private double targetVelocity;


    public static double FRONT_Kp =0.000002;
    public static double REAR_Kp = 0.000002;

    private double FRONT_Kf_a;
    private double FRONT_Kf_b;
    private double FRONT_Kf_c;
    private double REAR_Kf_a;
    private double REAR_Kf_b;
    private double REAR_Kf_c;

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
        frontShooterMotor =  hardwareMap.get(DcMotorEx.class, VelocityMotorConstantsBase.frontMotorName);
        MotorConfigurationType motorConfigurationType = frontShooterMotor.getMotorType().clone();
        rearShooterMotor = hardwareMap.get(DcMotorEx.class, VelocityMotorConstantsBase.rearMotorName);
        motorConfigurationType.setTicksPerRev(MotorConstants.ticksPerRev);
        motorConfigurationType.setGearing(MotorConstants.gearing);
        motorConfigurationType.setAchieveableMaxRPMFraction(MotorConstants.achievableMaxRPMFraction);

        frontShooterMotor.setMotorType(motorConfigurationType);
        frontShooterMotor.setMode(VelocityMotorConstantsBase.resetMode);
        frontShooterMotor.setMode(VelocityMotorConstantsBase.mode);
        frontShooterMotor.setDirection(VelocityMotorConstantsBase.frontMotorDirection);

        rearShooterMotor.setMotorType(motorConfigurationType);
        rearShooterMotor.setMode(VelocityMotorConstantsBase.resetMode);
        rearShooterMotor.setMode(VelocityMotorConstantsBase.mode);
        rearShooterMotor.setDirection(VelocityMotorConstantsBase.rearMotorDirection);
    }

    private void initializeLocalVariablesWithConstants(){
        START_VELOCITY = MotorConstants.startPosition;
        TOLERABLE_ERROR = MotorConstants.tolerableError;

        FRONT_Kf_a = VelocityTBHControllerConstantBase.FRONT_Kf_a;
        FRONT_Kf_b = VelocityTBHControllerConstantBase.FRONT_Kf_b;
        FRONT_Kf_c = VelocityTBHControllerConstantBase.FRONT_Kf_c;
        REAR_Kf_a = VelocityTBHControllerConstantBase.REAR_Kf_a;
        REAR_Kf_b = VelocityTBHControllerConstantBase.REAR_Kf_b;
        REAR_Kf_c = VelocityTBHControllerConstantBase.REAR_Kf_c;
    }

    private void initializeTBHController(){

        frontTbhController = new TBHController(FRONT_Kp, FRONT_Kf_a, FRONT_Kf_b, FRONT_Kf_c, telemetry);

        rearTbhController = new TBHController(REAR_Kp, REAR_Kf_a, REAR_Kf_b, REAR_Kf_c, telemetry);
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
