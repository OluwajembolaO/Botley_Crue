package org.firstinspires.ftc.teamcode.atHomeTest;

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
    private DcMotor turretMotor;

    // Add your drive motors here
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Auto-lock parameters
    private static final double SMOOTHNESS = 0.15;
    private static final double TOLERANCE = 5;
    private static final double MAX_POWER = 0.4;
    private static final double MIN_POWER = 0.1;

    // Smoothing state
    private double currentPower = 0.0;
    private double targetPower = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    // Toggle state for auto-lock
    private boolean autoLockActive = false;
    private boolean r3WasPressed = false;  // Track previous button state for toggle

    @Override
    public void init() {
        // Hardware initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        // Initialize drive motors (uncomment and adjust names as needed)
        // frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        // frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        // backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        // backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Configure turret motor
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        // Calculate delta time
        double deltaTime = timer.seconds();
        timer.reset();

        // === TOGGLE AUTO-LOCK WITH R3 ===
        boolean r3IsPressed = gamepad1.right_stick_button;

        // Detect button press (rising edge detection)
        if (r3IsPressed && !r3WasPressed) {
            // Toggle the auto-lock state
            autoLockActive = !autoLockActive;

            // Reset smoothing state when toggling off
            if (!autoLockActive) {
                currentPower = 0.0;
                targetPower = 0.0;
            }
        }
        r3WasPressed = r3IsPressed;

        // === DRIVE CODE (add your drive logic here) ===
        // Example mecanum drive:
        // double y = -gamepad1.left_stick_y;
        // double x = gamepad1.left_stick_x;
        // double rx = gamepad1.right_stick_x;
        //
        // frontLeft.setPower(y + x + rx);
        // frontRight.setPower(y - x - rx);
        // backLeft.setPower(y - x + rx);
        // backRight.setPower(y + x - rx);

        // === AUTO-LOCK MODE ===
        if (autoLockActive) {
            // Auto-lock is ACTIVE - track AprilTag
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();

                if (Math.abs(tx) > TOLERANCE) {
                    // Not aligned - adjust turret
                    targetPower = calculatePower(tx);
                    targetPower = clamp(targetPower, -MAX_POWER, MAX_POWER);

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

                    // Telemetry
                    telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
                    telemetry.addLine(tx > 0 ? ">>> TRACKING RIGHT >>>" : "<<< TRACKING LEFT <<<");
                    telemetry.addData("Error", "%.2f°", tx);
                    telemetry.addData("Applied Power", "%.3f", appliedPower);

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

                    telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
                    gamepad1.rumble(1); //Vibrate
                    telemetry.addLine("✓✓✓ LOCKED ON TARGET ✓✓✓");
                    telemetry.addData("Error", "%.2f°", tx);
                }

            } else {
                // No target - slow stop
                targetPower = 0.0;
                currentPower = currentPower * 0.7;

                if (Math.abs(currentPower) < 0.01) {
                    currentPower = 0.0;
                }

                turretMotor.setPower(-currentPower);

                telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
                telemetry.addLine("⚠ NO TARGET DETECTED ⚠");
            }

        } else {
            // Auto-lock is OFF - manual turret control
            double manualTurretControl = gamepad1.right_stick_x * 0.1; // Adjust sensitivity
            turretMotor.setPower(-manualTurretControl);

            // Reset smoothing state when not in auto-lock
            currentPower = 0.0;
            targetPower = 0.0;

            telemetry.addLine("Manual Control Mode");
            telemetry.addLine("Press R3 to enable Auto-Lock");
            telemetry.addData("Manual Turret Power", "%.2f", manualTurretControl);
        }

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
        turretMotor.setPower(0);
        limelight.stop();
    }
}