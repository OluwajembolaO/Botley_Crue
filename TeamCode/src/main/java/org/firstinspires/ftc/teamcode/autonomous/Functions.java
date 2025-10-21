package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Functions {
    DcMotor frontLeft, frontRight, rearLeft, rearRight;

    public Functions(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "motor1");
        frontRight = hardwareMap.get(DcMotor.class, "motor2");
        rearLeft = hardwareMap.get(DcMotor.class, "motor3");
        rearRight = hardwareMap.get(DcMotor.class, "motor4");

        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // ──────────────── Basic Movement ────────────────
    public void moveForward(double power, double seconds) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
        sleep(seconds);
        stop();
    }

    public void moveBackward(double power, double seconds) {
        moveForward(-power, seconds);
    }

    public void turnLeft(double power, double seconds) {
        frontLeft.setPower(-power);
        rearLeft.setPower(-power);
        frontRight.setPower(power);
        rearRight.setPower(power);
        sleep(seconds);
        stop();
    }

    public void turnRight(double power, double seconds) {
        turnLeft(-power, seconds);
    }

    public void moveRight(double power, double seconds){
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        rearLeft.setPower(-power);
        rearRight.setPower(power);
        sleep(seconds);
        stop();
    }

    public void moveLeft(double power, double seconds) {
        moveRight(-power, seconds);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    // ──────────────── Diagonal Movement ────────────────
    // Move diagonally forward-left (FL & RR move)
    public void moveDiagonalForwardLeft(double power, double seconds) {
        frontLeft.setPower(power);
        rearRight.setPower(power);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        sleep(seconds);
        stop();
    }

    // Move diagonally forward-right (FR & RL move)
    public void moveDiagonalForwardRight(double power, double seconds) {
        frontRight.setPower(power);
        rearLeft.setPower(power);
        frontLeft.setPower(0);
        rearRight.setPower(0);
        sleep(seconds);
        stop();
    }

    // Move diagonally backward-left (FR & RL move backward)
    public void moveDiagonalBackwardLeft(double power, double seconds) {
        frontRight.setPower(-power);
        rearLeft.setPower(-power);
        frontLeft.setPower(0);
        rearRight.setPower(0);
        sleep(seconds);
        stop();
    }

    // Move diagonally backward-right (FL & RR move backward)
    public void moveDiagonalBackwardRight(double power, double seconds) {
        frontLeft.setPower(-power);
        rearRight.setPower(-power);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        sleep(seconds);
        stop();
    }

    // ──────────────── Helper Methods ────────────────
    private void sleep(double seconds) {
        try {
            Thread.sleep((long)(seconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}