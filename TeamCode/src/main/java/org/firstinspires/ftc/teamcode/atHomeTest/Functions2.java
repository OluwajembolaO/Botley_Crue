package org.firstinspires.ftc.teamcode.atHomeTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Functions2 {
    DcMotor frontLeft, frontRight, rearLeft, rearRight;
    Limelight3A limelight3A;
    IMU imu;

    // Alignment parameters
    private static final double DEADZONE = 3.0;  // degrees - alignment tolerance
    private static final double SEARCH_POWER = 0.3;  // Power for searching
    private static final double ALIGN_POWER = 0.25;   // Power for fine alignment

    //Initializes everything
    public Functions2(HardwareMap hardwareMap) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft = hardwareMap.get(DcMotor.class, "motor1");
        frontRight = hardwareMap.get(DcMotor.class, "motor2");
        rearLeft = hardwareMap.get(DcMotor.class, "motor3");
        rearRight = hardwareMap.get(DcMotor.class, "motor4");

        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initializes IMU
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void startLL() { //Starts LimeLight
        limelight3A.start();
        limelight3A.pipelineSwitch(8); //April Tag
    }

    /**
     * Rotates the robot to search for a specific AprilTag
     * Keeps rotating until the AprilTag is detected
     * @param id The AprilTag ID to search for
     * @return true if AprilTag found, false if still searching
     */
    public boolean turnToAprilTag(int id) {
        // Update limelight with robot orientation
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight3A.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            if (llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {
                // Check if we found the specific AprilTag ID
                for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                    if (fiducial.getFiducialId() == id) {
                        // Found the target AprilTag! Stop rotating
                        stopMotors();
                        return true;
                    }
                }
            }
        }

        // No AprilTag found - keep rotating left continuously
        setTurnLeftPower(SEARCH_POWER);
        return false;
    }

    /**
     * Aligns the robot with a specific AprilTag for accurate shooting
     * Makes fine adjustments to center the AprilTag
     * @param id The AprilTag ID to align with
     * @return true if aligned within deadzone, false if still aligning
     */
    public boolean alignWithAprilTag(int id) {
        // Update limelight with robot orientation
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight3A.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            if (llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {

                // Search for the specific AprilTag ID
                for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                    if (fiducial.getFiducialId() == id) {
                        double tx = fiducial.getTargetXDegrees();

                        // Check if aligned within deadzone
                        if (Math.abs(tx) > DEADZONE) {
                            // Not aligned - make fine adjustments
                            if (tx > 0) {
                                // Tag is to the RIGHT - turn RIGHT (clockwise)
                                setTurnRightPower(ALIGN_POWER);
                            } else {
                                // Tag is to the LEFT - turn LEFT (counter-clockwise)
                                setTurnLeftPower(ALIGN_POWER);
                            }
                            return false;  // Still aligning
                        } else {
                            // Aligned! Stop motors
                            stopMotors();
                            return true;  // Alignment complete - ready to shoot!
                        }
                    }
                }
            }
        }

        // Lost the target - stop motors
        stopMotors();
        return false;
    }

    public void swapPipelineTo(int num){
        limelight3A.pipelineSwitch(num);
    }

    // ========== TIMED MOVEMENT METHODS ==========

    public void moveForward(double power, double seconds) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
        sleep(seconds);
        stopMotors();
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
        stopMotors();
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
        stopMotors();
    }

    public void moveLeft(double power, double seconds) {
        moveRight(-power, seconds);
    }

    // ========== CONTINUOUS MOVEMENT HELPERS (private) ==========

    private void setTurnLeftPower(double power) {
        frontLeft.setPower(-power);
        rearLeft.setPower(-power);
        frontRight.setPower(power);
        rearRight.setPower(power);
    }

    private void setTurnRightPower(double power) {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(-power);
        rearRight.setPower(-power);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    public void sleep(double seconds) {
        try {
            Thread.sleep((long)(seconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}