package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto")
public class Auto extends LinearOpMode {

    Functions robot;

    enum State {
        MOVE_FORWARD,
        MOVE_BACKWARD,
        TURN_LEFT,
        TURN_RIGHT,
        DIAGONAL_FL,
        DIAGONAL_BR,
        DIAGONAL_FR,
        DIAGONAL_BL,
        WAIT,
        STOP
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Functions(hardwareMap);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        State currentState = State.MOVE_FORWARD;
        State nextState = State.STOP;  // used for WAIT transitions
        long stateStartTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            long elapsed = System.currentTimeMillis() - stateStartTime;

            switch (currentState) {

                // ─────────── Move Forward ───────────
                case MOVE_FORWARD:
                    telemetry.addLine("State: MOVE_FORWARD (3 sec)");
                    telemetry.update();
                    robot.moveForward(0.5);

                    if (elapsed > 3000) {
                        robot.stop();
                        nextState = State.MOVE_BACKWARD;  // next action after wait
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Move Backward ───────────
                case MOVE_BACKWARD:
                    telemetry.addLine("State: MOVE_BACKWARD (3 sec)");
                    telemetry.update();
                    robot.moveBackward(0.5);

                    if (elapsed > 3000) {
                        robot.stop();
                        nextState = State.TURN_LEFT;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Turn Left ───────────
                case TURN_LEFT:
                    telemetry.addLine("State: TURN_LEFT (2 sec)");
                    telemetry.update();
                    robot.turnLeft(0.5);

                    if (elapsed > 2000) {
                        robot.stop();
                        nextState = State.TURN_RIGHT;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Turn Right ───────────
                case TURN_RIGHT:
                    telemetry.addLine("State: TURN_RIGHT (2 sec)");
                    telemetry.update();
                    robot.turnRight(0.5);

                    if (elapsed > 2000) {
                        robot.stop();
                        nextState = State.DIAGONAL_FL;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Diagonal Forward Left ───────────
                case DIAGONAL_FL:
                    telemetry.addLine("State: DIAGONAL_FORWARD_LEFT (2 sec)");
                    telemetry.update();
                    robot.moveDiagonalForwardLeft(0.5);

                    if (elapsed > 2000) {
                        robot.stop();
                        nextState = State.DIAGONAL_BR;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Diagonal Backward Right ───────────
                case DIAGONAL_BR:
                    telemetry.addLine("State: DIAGONAL_BACKWARD_RIGHT (2 sec)");
                    telemetry.update();
                    robot.moveDiagonalBackwardRight(0.5);

                    if (elapsed > 2000) {
                        robot.stop();
                        nextState = State.DIAGONAL_FR;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Diagonal Forward Right ───────────
                case DIAGONAL_FR:
                    telemetry.addLine("State: DIAGONAL_FORWARD_RIGHT (2 sec)");
                    telemetry.update();
                    robot.moveDiagonalForwardRight(0.5);

                    if (elapsed > 2000) {
                        robot.stop();
                        nextState = State.DIAGONAL_BL;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Diagonal Backward Left ───────────
                case DIAGONAL_BL:
                    telemetry.addLine("State: DIAGONAL_BACKWARD_LEFT (2 sec)");
                    telemetry.update();
                    robot.moveDiagonalBackwardLeft(0.5);

                    if (elapsed > 2000) {
                        robot.stop();
                        nextState = State.STOP;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── WAIT ───────────
                case WAIT:
                    telemetry.addLine("State: WAIT (1 second delay)");
                    telemetry.update();
                    robot.stop();

                    if (elapsed > 1000) { // 1 second pause
                        currentState = nextState;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── STOP ───────────
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
