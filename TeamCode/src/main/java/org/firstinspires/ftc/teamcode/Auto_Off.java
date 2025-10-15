package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Auto_Off extends LinearOpMode {

    // Declare motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    // Normalize and set motor powers
    private void setMotorPowers(double fl, double fr, double rl, double rr) {
        // Find the maximum absolute power
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(rl), Math.abs(rr)));

        // Normalize if any power exceeds 1.0
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            rl /= max;
            rr /= max;
        }

        // Set the motor powers
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        rearLeft.setPower(rl);
        rearRight.setPower(rr);
    }

    public void moveForward(double power) {
        setMotorPowers(power, power, power, power);
    }

    public void moveBackward(double power) {
        setMotorPowers(-power, -power, -power, -power);
    }

    public void strafeLeft(double power) {
        setMotorPowers(-power, power, power, -power);
    }

    public void strafeRight(double power) {
        setMotorPowers(power, -power, -power, power);
    }

    public void turnLeft(double power) {
        setMotorPowers(-power, power, -power, power);
    }

    public void turnRight(double power) {
        setMotorPowers(power, -power, power, -power);
    }

    public void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    // ──────────────── Diagonal Movement ────────────────
    // Mecanum diagonal: different wheels have different powers
    public void moveDiagonalForwardLeft(double power) {
        setMotorPowers(power, 0, 0, power);
    }

    public void moveDiagonalForwardRight(double power) {
        setMotorPowers(0, power, power, 0);
    }

    public void moveDiagonalBackwardLeft(double power) {
        setMotorPowers(0, -power, -power, 0);
    }

    public void moveDiagonalBackwardRight(double power) {
        setMotorPowers(-power, 0, 0, -power);
    }

    enum State {
        MOVE_FORWARD,
        MOVE_BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT,
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
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "motor1");
        frontRight = hardwareMap.get(DcMotor.class, "motor2");
        rearLeft = hardwareMap.get(DcMotor.class, "motor3");
        rearRight = hardwareMap.get(DcMotor.class, "motor4");

        // IMPORTANT: Reverse right-side motors for mecanum drive
        // Adjust these based on your robot's configuration
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        State currentState = State.MOVE_FORWARD;
        State nextState = State.STOP;
        long stateStartTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            long elapsed = System.currentTimeMillis() - stateStartTime;

            switch (currentState) {

                case MOVE_FORWARD:
                    telemetry.addLine("State: MOVE_FORWARD (3 sec)");
                    telemetry.update();
                    moveForward(0.8);

                    if (elapsed > 3000) {
                        stopMotors();
                        nextState = State.MOVE_BACKWARD;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case MOVE_BACKWARD:
                    telemetry.addLine("State: MOVE_BACKWARD (3 sec)");
                    telemetry.update();
                    moveBackward(0.8);

                    if (elapsed > 3000) {
                        stopMotors();
                        nextState = State.STRAFE_LEFT;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case STRAFE_LEFT:
                    telemetry.addLine("State: STRAFE_LEFT (2 sec)");
                    telemetry.update();
                    strafeLeft(0.8);

                    if (elapsed > 2000) {
                        stopMotors();
                        nextState = State.STRAFE_RIGHT;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case STRAFE_RIGHT:
                    telemetry.addLine("State: STRAFE_RIGHT (2 sec)");
                    telemetry.update();
                    strafeRight(0.8);

                    if (elapsed > 2000) {
                        stopMotors();
                        nextState = State.TURN_LEFT;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case TURN_LEFT:
                    telemetry.addLine("State: TURN_LEFT (2 sec)");
                    telemetry.update();
                    turnLeft(0.8);

                    if (elapsed > 2000) {
                        stopMotors();
                        nextState = State.TURN_RIGHT;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case TURN_RIGHT:
                    telemetry.addLine("State: TURN_RIGHT (2 sec)");
                    telemetry.update();
                    turnRight(0.8);

                    if (elapsed > 2000) {
                        stopMotors();
                        nextState = State.DIAGONAL_FL;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case DIAGONAL_FL:
                    telemetry.addLine("State: DIAGONAL_FORWARD_LEFT (2 sec)");
                    telemetry.update();
                    moveDiagonalForwardLeft(0.8);

                    if (elapsed > 2000) {
                        stopMotors();
                        nextState = State.DIAGONAL_BR;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case DIAGONAL_BR:
                    telemetry.addLine("State: DIAGONAL_BACKWARD_RIGHT (2 sec)");
                    telemetry.update();
                    moveDiagonalBackwardRight(0.8);

                    if (elapsed > 2000) {
                        stopMotors();
                        nextState = State.DIAGONAL_FR;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case DIAGONAL_FR:
                    telemetry.addLine("State: DIAGONAL_FORWARD_RIGHT (2 sec)");
                    telemetry.update();
                    moveDiagonalForwardRight(0.8);

                    if (elapsed > 2000) {
                        stopMotors();
                        nextState = State.DIAGONAL_BL;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case DIAGONAL_BL:
                    telemetry.addLine("State: DIAGONAL_BACKWARD_LEFT (2 sec)");
                    telemetry.update();
                    moveDiagonalBackwardLeft(0.8);

                    if (elapsed > 2000) {
                        stopMotors();
                        nextState = State.STOP;
                        currentState = State.WAIT;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case WAIT:
                    telemetry.addLine("State: WAIT (1 second delay)");
                    telemetry.update();
                    stopMotors();

                    if (elapsed > 1000) {
                        currentState = nextState;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case STOP:
                    telemetry.addLine("State: STOP (Autonomous Complete)");
                    telemetry.update();
                    stopMotors();
                    return;
            }
            sleep(50);
        }
    }
}