package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.actuators.elevator;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.elevator.ElevatorController;

public class PushElevator implements ActionFunction {
    Telemetry telemetry;
    ElevatorController elevatorController;

    protected Status lastStatus = Status.FAILURE;


    boolean started = false;


    public PushElevator(Telemetry telemetry, ElevatorController elevatorController) {
        this.telemetry = telemetry;
        this.elevatorController = elevatorController;
        this.init();
    }

    private void init() {

    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        double targetPosition = 100;

        if (!started) {
            elevatorController.moveToTargetPosition(targetPosition, Servo.Direction.FORWARD);
            started = true;
            status = Status.RUNNING;
        } else {
                status = Status.SUCCESS;
        }

        elevatorController.update();
        lastStatus = status;
        return status;
    }
}
