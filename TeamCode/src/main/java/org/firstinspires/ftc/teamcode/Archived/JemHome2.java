package org.firstinspires.ftc.teamcode.Archived;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="AutoLock TeleOp")
public class JemHome2 extends OpMode {

    // Hardware
    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Auto-lock parameters
    private static final double SMOOTHNESS = 0.15;
    private static final double TOLERANCE = 3;
    private static final double MAX_POWER = 0.4;
    private static final double MIN_POWER = 0.1;

    // Smoothing state
    private double currentPower = 0.0;
    private double targetPower = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    // Toggle state for auto-lock
    private boolean autoLockActive = false;
    private boolean r3WasPressed = false;

    @Override
    public void init() {
        // Hardware initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions (adjust based on your robot's configuration)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Configure motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("TeleOp with Auto-Lock Ready");
        telemetry.addLine("Press R3 (Right Stick Button) to toggle auto-lock");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.pipelineSwitch(8);
        limelight.start();
        timer.reset();
    }

    @Override
    public void loop() {
        double deltaTime = timer.seconds();
        timer.reset();

        // === TOGGLE AUTO-LOCK WITH R3 ===
        boolean r3IsPressed = gamepad1.right_stick_button;

        if (r3IsPressed && !r3WasPressed) {
            autoLockActive = !autoLockActive;

            if (!autoLockActive) {
                currentPower = 0.0;
                targetPower = 0.0;
            }
        }
        r3WasPressed = r3IsPressed;

        // === DRIVER INPUT ===
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x;   // Strafe
        double rx = gamepad1.right_stick_x; // Rotation (manual)

        // === AUTO-LOCK MODE ===
        double autoRotation = 0.0;

        if (autoLockActive) {
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();

                if (Math.abs(tx) > TOLERANCE) {
                    // Not aligned - calculate rotation power
                    targetPower = calculatePower(tx);
                    targetPower = clamp(targetPower, -MAX_POWER, MAX_POWER);

                    // Smooth interpolation
                    double smoothFactor = SMOOTHNESS * (deltaTime * 60.0);
                    smoothFactor = Math.min(smoothFactor, 1.0);
                    currentPower = currentPower + (targetPower - currentPower) * smoothFactor;

                    // Apply minimum power
                    autoRotation = currentPower;
                    if (Math.abs(currentPower) < MIN_POWER && Math.abs(targetPower) > MIN_POWER) {
                        autoRotation = Math.signum(currentPower) * MIN_POWER;
                    }

                    telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
                    telemetry.addLine(tx > 0 ? ">>> TRACKING RIGHT >>>" : "<<< TRACKING LEFT <<<");
                    telemetry.addData("Error", "%.2f°", tx);
                    telemetry.addData("Auto Rotation", "%.3f", autoRotation);

                } else {
                    // Aligned!
                    targetPower = 0.0;
                    currentPower = currentPower * 0.8;

                    if (Math.abs(currentPower) < 0.01) {
                        currentPower = 0.0;
                    }
                    autoRotation = currentPower;

                    telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
                    gamepad1.rumble(1);
                    telemetry.addLine("✓✓✓ LOCKED ON TARGET ✓✓✓");
                    telemetry.addData("Error", "%.2f°", tx);
                }

            } else {
                // No target - slow stop rotation
                targetPower = 0.0;
                currentPower = currentPower * 0.7;

                if (Math.abs(currentPower) < 0.01) {
                    currentPower = 0.0;
                }
                autoRotation = currentPower;

                telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
                telemetry.addLine("⚠ NO TARGET DETECTED ⚠");
            }

        } else {
            // Manual control - use right stick for rotation
            currentPower = 0.0;
            targetPower = 0.0;

            telemetry.addLine("Manual Control Mode");
            telemetry.addLine("Press R3 to enable Auto-Lock");
        }

        // === APPLY MOTOR POWERS ===
        // Use auto-rotation when auto-lock is active, otherwise use manual rotation
        double rotation = autoLockActive ? autoRotation : rx;

        // Mecanum drive calculation
        double frontLeftPower = y + x + rotation;
        double frontRightPower = y - x - rotation;
        double backLeftPower = y - x + rotation;
        double backRightPower = y + x - rotation;

        // Normalize powers if any exceeds 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower),
                                Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Status display
        telemetry.addLine();
        telemetry.addData("Auto-Lock Status", autoLockActive ? "🔒 ACTIVE" : "OFF");
        telemetry.addData("FPS", "%.1f", 1.0 / Math.max(deltaTime, 0.001));
        telemetry.update();
    }

    private double calculatePower(double error) {
        double normalizedError = Math.abs(error) / 30.0;
        normalizedError = Math.min(normalizedError, 1.0);

        double linearComponent = normalizedError * 0.4;
        double squaredComponent = normalizedError * normalizedError * 0.6;
        double magnitude = (linearComponent + squaredComponent) * MAX_POWER;

        return Math.signum(error) * magnitude;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        limelight.stop();
    }
}