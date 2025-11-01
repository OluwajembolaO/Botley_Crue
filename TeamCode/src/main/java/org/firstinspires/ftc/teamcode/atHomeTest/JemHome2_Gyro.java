package org.firstinspires.ftc.teamcode.atHomeTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="TurretAutoLock")
public class JemHome2_Gyro extends OpMode {

    // Hardware
    private Limelight3A limelight;
    private DcMotor turretMotor;
    private IMU imu;

    // Smooth interpolation parameters (Roblox-style lerping)
    private static final double SMOOTHNESS = 0.25;  // Higher for gyro-stabilized tracking
    private static final double TOLERANCE = 1.0;  // degrees - aligned when within this
    private static final double MAX_POWER = 0.6;  // Maximum motor power
    private static final double MIN_POWER = 0.02;  // Minimum power to overcome friction

    // Gyroscopic stabilization (chicken head effect)
    private double lastRobotYaw = 0.0;  // Previous robot heading
    private double turretTargetAngle = 0.0;  // Absolute target angle in world space
    private boolean hasTarget = false;

    // Smoothing state
    private double currentPower = 0.0;
    private double targetPower = 0.0;  // Target motor power (from error)
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        // Hardware initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        imu = hardwareMap.get(IMU.class, "imu");

        // Configure turret motor
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU setup (built into Control Hub)
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        telemetry.addLine("Turret Auto-Lock Ready");
        telemetry.addLine("Gyro-stabilized (Chicken Head Mode)");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.pipelineSwitch(8);
        limelight.start();
        timer.reset();
        lastRobotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Override
    public void loop() {
        double deltaTime = timer.seconds();
        timer.reset();

        // Get current robot orientation
        double currentRobotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double robotRotationDelta = currentRobotYaw - lastRobotYaw;

        // Handle angle wrap-around
        if (robotRotationDelta > 180) robotRotationDelta -= 360;
        if (robotRotationDelta < -180) robotRotationDelta += 360;

        lastRobotYaw = currentRobotYaw;

        // Get Limelight result
        LLResult llResult = limelight.getLatestResult();

        telemetry.clear();
        telemetry.addLine("=== GYRO-STABILIZED TURRET ===");
        telemetry.addLine();

        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();

            // Update target angle in world space
            // This is the "chicken head" magic: we remember where the target is
            turretTargetAngle = tx; // In camera-relative space
            hasTarget = true;

            telemetry.addData("Target Offset (TX)", "%.2f°", tx);
            telemetry.addData("Robot Yaw", "%.2f°", currentRobotYaw);

            // Calculate error (how far turret needs to rotate)
            double error = tx;

            if (Math.abs(error) > TOLERANCE) {
                // Calculate desired power
                double targetPower = calculatePower(error);
                targetPower = clamp(targetPower, -MAX_POWER, MAX_POWER);

                // Add compensation for robot rotation (CHICKEN HEAD EFFECT)
                // If robot rotates right, turret must rotate left to stay locked
                double gyroCompensation = -robotRotationDelta * 0.05; // Scale factor
                targetPower += gyroCompensation;

                // Smooth interpolation
                double smoothFactor = SMOOTHNESS * (deltaTime * 60.0);
                smoothFactor = Math.min(smoothFactor, 1.0);
                currentPower = currentPower + (targetPower - currentPower) * smoothFactor;

                // Apply minimum power
                double appliedPower = currentPower;
                if (Math.abs(currentPower) < MIN_POWER && Math.abs(targetPower) > MIN_POWER) {
                    appliedPower = Math.signum(currentPower) * MIN_POWER;
                }

                turretMotor.setPower(-appliedPower);

                telemetry.addLine(tx > 0 ? ">>> TRACKING RIGHT >>>" : "<<< TRACKING LEFT <<<");
                telemetry.addData("Error", "%.2f°", error);
                telemetry.addData("Gyro Compensation", "%.3f", gyroCompensation);
                telemetry.addData("Applied Power", "%.3f", appliedPower);

            } else {
                // Target centered - hold position with gyro compensation
                double gyroCompensation = -robotRotationDelta * 0.05;
                currentPower = currentPower * 0.9 + gyroCompensation;

                if (Math.abs(currentPower) < 0.01 && Math.abs(robotRotationDelta) < 0.5) {
                    currentPower = 0.0;
                }

                turretMotor.setPower(-currentPower);

                telemetry.addLine("✓✓✓ LOCKED ON TARGET ✓✓✓");
                telemetry.addData("Holding with gyro stabilization", "");
            }

        } else {
            // No target - maintain last known angle with gyro compensation
            if (hasTarget) {
                // Keep turret pointed at last known position
                double gyroCompensation = -robotRotationDelta * 0.05;
                currentPower = currentPower * 0.85 + gyroCompensation;

                if (Math.abs(currentPower) < 0.01) {
                    currentPower = 0.0;
                }

                turretMotor.setPower(-currentPower);

                telemetry.addLine("⚠ TARGET LOST - HOLDING POSITION ⚠");
                telemetry.addData("Gyro-stabilized hold", "");
            } else {
                // Never had a target
                targetPower = 0.0;
                currentPower = currentPower * 0.7;

                if (Math.abs(currentPower) < 0.01) {
                    currentPower = 0.0;
                }

                turretMotor.setPower(-currentPower);

                telemetry.addLine("⚠ SEARCHING FOR TARGET ⚠");
            }
        }

        telemetry.addData("Robot Rotation Rate", "%.2f°/s", robotRotationDelta / Math.max(deltaTime, 0.001));
        telemetry.addData("FPS", "%.1f", 1.0 / Math.max(deltaTime, 0.001));
        telemetry.update();
    }

    private double calculatePower(double error) {
        double normalizedError = Math.abs(error) / 30.0;
        normalizedError = Math.min(normalizedError, 1.0);

        // Smoother curve for stabilized tracking
        double linearComponent = normalizedError * 0.3;
        double squaredComponent = normalizedError * normalizedError * 0.7;
        double magnitude = (linearComponent + squaredComponent) * MAX_POWER;

        return Math.signum(error) * magnitude;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        turretMotor.setPower(0);
        limelight.stop();
    }
}