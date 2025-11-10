package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.actuators.carousel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.carousel.CarouselController;

public class TurnPortToShoot implements ActionFunction {
    Telemetry telemetry;
    CarouselController carouselController;

    protected Status lastStatus = Status.FAILURE;


    boolean started = false;


    public TurnPortToShoot(Telemetry telemetry, CarouselController carouselController) {
        this.telemetry = telemetry;
        this.carouselController = carouselController;
        this.init();
    }

    private void init() {


    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        double targetPosition = (double) blackBoard.getValue("CarouselTargetPosition");

        if (!started) {
            carouselController.moveToTargetPosition(targetPosition);
            started = true;
            status = Status.RUNNING;
        } else {
            if (carouselController.isBusy()) {
                status = Status.RUNNING;
            } else {
                    status = Status.SUCCESS;
            }
        }

        carouselController.update();
        lastStatus = status;
        return status;
    }

}
