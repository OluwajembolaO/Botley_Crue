package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Functions {
    DcMotor frontLeft, frontRight, rearLeft, rearRight;

    public Functions(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "motor1");
        frontRight = hardwareMap.get(DcMotor.class, "motor2");
        rearLeft = hardwareMap.get(DcMotor.class, "motor3");
        rearRight = hardwareMap.get(DcMotor.class, "motor4");
    }

    // ──────────────── Basic Movement ────────────────
    public void moveForward(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
    }

    public void moveBackward(double power) {
        moveForward(-power);
    }

    public void turnLeft(double power) {
        frontLeft.setPower(-power);
        rearLeft.setPower(-power);
        frontRight.setPower(power);
        rearRight.setPower(power);
    }

    public void turnRight(double power) {
        turnLeft(-power);
    }

    public void stop() {
        moveForward(0);
    }

    // ──────────────── Diagonal Movement ────────────────
    // Move diagonally forward-left (FL & RR move)
    public void moveDiagonalForwardLeft(double power) {
        frontLeft.setPower(power);
        rearRight.setPower(power);
        frontRight.setPower(0);
        rearLeft.setPower(0);
    }

    // Move diagonally forward-right (FR & RL move)
    public void moveDiagonalForwardRight(double power) {
        frontRight.setPower(power);
        rearLeft.setPower(power);
        frontLeft.setPower(0);
        rearRight.setPower(0);
    }

    // Move diagonally backward-left (FR & RL move backward)
    public void moveDiagonalBackwardLeft(double power) {
        frontRight.setPower(-power);
        rearLeft.setPower(-power);
        frontLeft.setPower(0);
        rearRight.setPower(0);
    }

    // Move diagonally backward-right (FL & RR move backward)
    public void moveDiagonalBackwardRight(double power) {
        frontLeft.setPower(-power);
        rearRight.setPower(-power);
        frontRight.setPower(0);
        rearLeft.setPower(0);
    }
}
