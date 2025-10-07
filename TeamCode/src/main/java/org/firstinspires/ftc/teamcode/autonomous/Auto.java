package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto")
public class Auto extends LinearOpMode {

    Functions robot;

    enum State {
        MOVE_FORWARD,
        WAIT1,
        MOVE_BACKWARD,
        WAIT2,
        TURN_LEFT,
        TURN_RIGHT,
        DIAGONAL_FL,
        DIAGONAL_BR,
        DIAGONAL_FR,
        DIAGONAL_BL,
        STOP
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Functions(hardwareMap);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        State currentState = State.MOVE_FORWARD;
        long stateStartTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            long elapsed = System.currentTimeMillis() - stateStartTime;

            switch (currentState) {

                case MOVE_FORWARD:
                    telemetry.addLine("State: MOVE_FORWARD (3 seconds)");
                    telemetry.update();
                    robot.moveForward(0.5);
                    if (elapsed > 3000) {
                        robot.stop();
                        currentState = State.WAIT1;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case WAIT1:
                    telemetry.addLine("State: WAIT1 (1 second)");
                    telemetry.update();
                    if (elapsed > 1000) {
                        currentState = State.MOVE_BACKWARD;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case MOVE_BACKWARD:
                    telemetry.addLine("State: MOVE_BACKWARD (3 seconds)");
                    telemetry.update();
                    robot.moveBackward(0.5);
                    if (elapsed > 3000) {
                        robot.stop();
                        currentState = State.WAIT2;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case WAIT2:
                    telemetry.addLine("State: WAIT2 (1 second)");
                    telemetry.update();
                    if (elapsed > 1000) {
                        currentState = State.TURN_LEFT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case TURN_LEFT:
                    telemetry.addLine("State: TURN_LEFT (2 seconds)");
                    telemetry.update();
                    robot.turnLeft(0.5);
                    if (elapsed > 2000) {
                        robot.stop();
                        currentState = State.TURN_RIGHT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case TURN_RIGHT:
                    telemetry.addLine("State: TURN_RIGHT (2 seconds)");
                    telemetry.update();
                    robot.turnRight(0.5);
                    if (elapsed > 2000) {
                        robot.stop();
                        currentState = State.DIAGONAL_FL;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case DIAGONAL_FL:
                    telemetry.addLine("State: DIAGONAL_FORWARD_LEFT (2 seconds)");
                    telemetry.update();
                    robot.moveDiagonalForwardLeft(0.5);
                    if (elapsed > 2000) {
                        robot.stop();
                        currentState = State.DIAGONAL_BR;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case DIAGONAL_BR:
                    telemetry.addLine("State: DIAGONAL_BACKWARD_RIGHT (2 seconds)");
                    telemetry.update();
                    robot.moveDiagonalBackwardRight(0.5);
                    if (elapsed > 2000) {
                        robot.stop();
                        currentState = State.DIAGONAL_FR;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case DIAGONAL_FR:
                    telemetry.addLine("State: DIAGONAL_FORWARD_RIGHT (2 seconds)");
                    telemetry.update();
                    robot.moveDiagonalForwardRight(0.5);
                    if (elapsed > 2000) {
                        robot.stop();
                        currentState = State.DIAGONAL_BL;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case DIAGONAL_BL:
                    telemetry.addLine("State: DIAGONAL_BACKWARD_LEFT (2 seconds)");
                    telemetry.update();
                    robot.moveDiagonalBackwardLeft(0.5);
                    if (elapsed > 2000) {
                        robot.stop();
                        currentState = State.STOP;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case STOP:
                    telemetry.addLine("State: STOP (Autonomous Complete)");
                    telemetry.update();
                    robot.stop();
                    return;
            }

            sleep(50);
        }
    }
}
