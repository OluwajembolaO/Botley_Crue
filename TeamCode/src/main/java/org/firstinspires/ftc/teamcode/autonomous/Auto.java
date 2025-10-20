package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto")
public class Auto extends LinearOpMode {

    Functions robot;

    //Different Actions
    enum State {
        MOVE_FORWARD_1,
        MOVE_BACKWARD,
        MOVE_FORWARD_2,
        SLIDE_RIGHT,
        SLIDE_LEFT,
        TURN_RIGHT,
        TURN_LEFT,
        WAIT,
        STOP
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Functions(hardwareMap);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        State currentState = State.MOVE_FORWARD_1;
        State nextState = State.STOP;  // used for WAIT transitions
        long stateStartTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            long elapsed = System.currentTimeMillis() - stateStartTime;

            switch (currentState) {

                // ─────────── Move Forward (4 sec) ───────────
                case MOVE_FORWARD_1:
                    telemetry.addLine("State: MOVE_FORWARD (4 sec)");
                    telemetry.update();
                    robot.moveForward(0.75);

                    if (elapsed > 4000) {
                        robot.stop();
                        nextState = State.MOVE_BACKWARD;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Move Backward (4 sec) ───────────
                case MOVE_BACKWARD:
                    telemetry.addLine("State: MOVE_BACKWARD (4 sec)");
                    telemetry.update();
                    robot.moveBackward(0.75);

                    if (elapsed > 4000) {
                        robot.stop();
                        nextState = State.MOVE_FORWARD_2;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Move Forward (2 sec) ───────────
                case MOVE_FORWARD_2:
                    telemetry.addLine("State: MOVE_FORWARD (2 sec)");
                    telemetry.update();
                    robot.moveForward(0.75);

                    if (elapsed > 2000) {
                        robot.stop();
                        nextState = State.SLIDE_RIGHT;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Slide Right (3 sec) ───────────
                case SLIDE_RIGHT:
                    telemetry.addLine("State: SLIDE_RIGHT (3 sec)");
                    telemetry.update();
                    robot.moveRight(0.75);

                    if (elapsed > 3000) {
                        robot.stop();
                        nextState = State.SLIDE_LEFT;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Slide Left (3 sec) ───────────
                case SLIDE_LEFT:
                    telemetry.addLine("State: SLIDE_LEFT (3 sec)");
                    telemetry.update();
                    robot.moveLeft(0.75);

                    if (elapsed > 3000) {
                        robot.stop();
                        nextState = State.TURN_RIGHT;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Turn Right (2 sec) ───────────
                case TURN_RIGHT:
                    telemetry.addLine("State: TURN_RIGHT (2 sec)");
                    telemetry.update();
                    robot.turnRight(0.75);

                    if (elapsed > 2000) {
                        robot.stop();
                        nextState = State.TURN_LEFT;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                // ─────────── Turn Left (2 sec) ───────────
                case TURN_LEFT:
                    telemetry.addLine("State: TURN_LEFT (2 sec)");
                    telemetry.update();
                    robot.turnLeft(0.75);

                    if (elapsed > 3500) {
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