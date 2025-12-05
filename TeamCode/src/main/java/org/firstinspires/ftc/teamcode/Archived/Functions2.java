package org.firstinspires.ftc.teamcode.Archived;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Functions2 {
    DcMotor frontLeft, frontRight, rearLeft, rearRight;
    DcMotor turretMotor;
    Limelight3A limelight3A;
    IMU imu;

    // Alignment parameters
    private static final double DEADZONE = 3.0;  // degrees - alignment tolerance
    private static final double SEARCH_POWER = 0.3;  // Power for searching
    private static final double ALIGN_POWER = 0.25;   // Power for fine alignment

    // Auto-lock parameters
    private static final double AUTOLOCK_SMOOTHNESS = 0.15;
    private static final double AUTOLOCK_TOLERANCE = 5;
    private static final double AUTOLOCK_MAX_POWER = 0.4;
    private static final double AUTOLOCK_MIN_POWER = 0.1;

    // Auto-lock smoothing state
    private double currentPower = 0.0;
    private double targetPower = 0.0;

    //Initializes everything
    public Functions2(HardwareMap hardwareMap) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft = hardwareMap.get(DcMotor.class, "motor1");
        frontRight = hardwareMap.get(DcMotor.class, "motor2");
        rearLeft = hardwareMap.get(DcMotor.class, "motor3");
        rearRight = hardwareMap.get(DcMotor.class, "motor4");

        // Initialize turret motor (optional - comment out if not using)
        try {
            turretMotor = hardwareMap.get(DcMotor.class, "turret");
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            turretMotor = null;  // Turret not configured
        }

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

    // ========== TURRET AUTO-LOCK FUNCTIONS ==========

    /**
     * Auto-lock turret function - tracks specific AprilTag and adjusts turret
     * Call this continuously while you want auto-lock active
     * @param id The AprilTag ID to lock onto
     * @param deltaTime Time since last call (in seconds)
     * @param telemetry Telemetry object for debug output (can be null)
     * @return true if locked on target, false otherwise
     */
    public boolean runTurretAutoLock(int id, double deltaTime, Telemetry telemetry) {
        if (turretMotor == null) {
            if (telemetry != null) {
                telemetry.addLine("⚠ TURRET NOT CONFIGURED ⚠");
            }
            return false;
        }

        LLResult llResult = limelight3A.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            if (llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {

                // Search for the specific AprilTag ID
                for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                    if (fiducial.getFiducialId() == id) {
                        double tx = fiducial.getTargetXDegrees();

                        if (Math.abs(tx) > AUTOLOCK_TOLERANCE) {
                            // Not aligned - adjust turret
                            targetPower = calculateAutolockPower(tx);
                            targetPower = clamp(targetPower, -AUTOLOCK_MAX_POWER, AUTOLOCK_MAX_POWER);

                            // Smooth interpolation
                            double smoothFactor = AUTOLOCK_SMOOTHNESS * (deltaTime * 60.0);
                            smoothFactor = Math.min(smoothFactor, 1.0);
                            currentPower = currentPower + (targetPower - currentPower) * smoothFactor;

                            // Apply minimum power
                            double appliedPower = currentPower;
                            if (Math.abs(currentPower) < AUTOLOCK_MIN_POWER && Math.abs(targetPower) > AUTOLOCK_MIN_POWER) {
                                appliedPower = Math.signum(currentPower) * AUTOLOCK_MIN_POWER;
                            }

                            turretMotor.setPower(-appliedPower);

                            // Telemetry
                            if (telemetry != null) {
                                telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
                                telemetry.addData("Target ID", id);
                                telemetry.addLine(tx > 0 ? ">>> TRACKING RIGHT >>>" : "<<< TRACKING LEFT <<<");
                                telemetry.addData("Error", "%.2f°", tx);
                                telemetry.addData("Applied Power", "%.3f", appliedPower);
                            }
                            return false;

                        } else {
                            // Aligned!
                            targetPower = 0.0;
                            currentPower = currentPower * 0.8;

                            if (Math.abs(currentPower) < 0.01) {
                                currentPower = 0.0;
                                turretMotor.setPower(0);
                            } else {
                                turretMotor.setPower(-currentPower);
                            }

                            if (telemetry != null) {
                                telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
                                telemetry.addData("Target ID", id);
                                telemetry.addLine("✓✓✓ LOCKED ON TARGET ✓✓✓");
                                telemetry.addData("Error", "%.2f°", tx);
                            }
                            return true;
                        }
                    }
                }
            }
        }

        // No target found with matching ID - slow stop
        targetPower = 0.0;
        currentPower = currentPower * 0.7;

        if (Math.abs(currentPower) < 0.01) {
            currentPower = 0.0;
        }

        turretMotor.setPower(-currentPower);

        if (telemetry != null) {
            telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
            telemetry.addLine("⚠ NO TARGET DETECTED ⚠");
        }
        return false;
    }
    /**
     * Manual turret control
     * @param power Power to apply to turret (-1.0 to 1.0)
     */
    public void runManualTurret(double power) {
        if (turretMotor != null) {
            turretMotor.setPower(power);
            // Reset smoothing state
            currentPower = 0.0;
            targetPower = 0.0;
        }
    }

    /**
     * Stop the turret motor
     */
    public void stopTurret() {
        if (turretMotor != null) {
            turretMotor.setPower(0);
            currentPower = 0.0;
            targetPower = 0.0;
        }
    }

    /**
     * Reset auto-lock state (call when switching from auto to manual)
     */
    public void resetAutoLockState() {
        currentPower = 0.0;
        targetPower = 0.0;
    }

    private double calculateAutolockPower(double error) {
        double normalizedError = Math.abs(error) / 30.0;
        normalizedError = Math.min(normalizedError, 1.0);

        double linearComponent = normalizedError * 0.4;
        double squaredComponent = normalizedError * normalizedError * 0.6;
        double magnitude = (linearComponent + squaredComponent) * AUTOLOCK_MAX_POWER;

        return Math.signum(error) * magnitude;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

// ========== PIPELINE SWITCHING ==========

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

