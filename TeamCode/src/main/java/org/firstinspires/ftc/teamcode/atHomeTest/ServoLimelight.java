package org.firstinspires.ftc.teamcode.atHomeTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo AutoLock TeleOp")
public class ServoLimelight extends OpMode {

    // Hardware
    private Limelight3A limelight;
    private Servo turretServo;

    // Drive motors
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    // Intake and outtake motors
    private DcMotor intake;
    private DcMotor outtake1;
    private DcMotor outtake2;

    // Auto-lock parameters
    private static final double SMOOTHNESS = 0.15;
    private static final double TOLERANCE = 5.0; // degrees
    private static final double MAX_SERVO_SPEED = 0.02; // Max change per update (TUNE THIS - start conservative)
    private static final double MIN_SERVO_SPEED = 0.003; // Min change per update

    // Servo range - 5-turn servo allows 0.0 to 1.0 for full 5 rotations
    // For 360° turntable rotation, you'll use a fraction based on your gear ratio
    // IMPORTANT: Adjust these after testing to match your actual gear ratio
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_CENTER = 0.5; // Starting center position

    // Gear ratio compensation flag
    // TRUE = servo direction matches desired rotation
    // FALSE = gears reverse the direction (your case - turret moves opposite of servo)
    private static final boolean REVERSE_DIRECTION = true;

    // Smoothing state
    private double currentPosition = SERVO_CENTER;
    private double targetPosition = SERVO_CENTER;
    private ElapsedTime timer = new ElapsedTime();

    // Toggle state for auto-lock
    private boolean autoLockActive = false;
    private boolean r3WasPressed = false;  // Track previous button state for toggle

    @Override
    public void init() {
        // Hardware initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(Servo.class, "turretServo");

        // Initialize drive motors
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        rearRightMotor = hardwareMap.get(DcMotor.class, "motor4");

        // Initialize intake and outtake motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");

        // Right side motors naturally go backwards so we flip em
        topRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Make one outtake spin the other way so they work together
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake behavior
        topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set servo to center position
        turretServo.setPosition(SERVO_CENTER);
        currentPosition = SERVO_CENTER;

        telemetry.addLine("TeleOp with Servo Auto-Lock Ready");
        telemetry.addLine("Press R3 (Right Stick Button) to toggle auto-lock");
        telemetry.addLine();
        telemetry.addLine("TUNING NOTES:");
        telemetry.addData("Reverse Direction", REVERSE_DIRECTION ? "YES (gears reverse motion)" : "NO");
        telemetry.addLine("Test rotation direction first!");
        telemetry.addLine("Adjust MAX_SERVO_SPEED if too fast/slow");
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

            // Set target to current position when toggling
            if (autoLockActive) {
                targetPosition = currentPosition;
            }
        }
        r3WasPressed = r3IsPressed;

        // === DRIVE CODE ===
        // Left bumper = intake go
        intake.setPower(0);
        if (gamepad1.left_bumper) {
            intake.setPower(1);
        }

        // Right bumper = outtake go brrr
        outtake1.setPower(0);
        outtake2.setPower(0);
        if (gamepad1.right_bumper) {
            outtake1.setPower(1);
            outtake2.setPower(1);  // spins opposite cause of setDirection above
        }

        double speed = 0.65; // normal driving speed
        double fwd = -gamepad1.left_stick_x; // forward/back
        double str = gamepad1.left_stick_y;  // side to side (strafe)
        double rot = -gamepad1.right_stick_x; // spinning around

        float lt = gamepad1.left_trigger;   // slow mode trigger
        float rt = gamepad1.right_trigger;  // fast mode trigger

        // Triggers make you go slower or faster
        if (lt > 0.5) {
            speed = 0.4; // chill mode activated
        }
        if (rt > 0.5) {
            speed = 0.9; // zoom zoom mode
        }

        // Mecanum drive math
        double tLPower = fwd + str + rot;
        double rLPower = fwd - str - rot;
        double tRPower = fwd - str + rot;
        double rRPower = fwd + str - rot;

        // Makes sure motors don't try to go over 100%
        double max = Math.max(1.0, Math.abs(tLPower));
        max = Math.max(max, Math.abs(rLPower));
        max = Math.max(max, Math.abs(tRPower));
        max = Math.max(max, Math.abs(rRPower));

        // Actually send power to the motors
        topLeftMotor.setPower((tLPower / max) * speed);
        topRightMotor.setPower((tRPower / max) * speed);
        rearLeftMotor.setPower((rLPower / max) * speed);
        rearRightMotor.setPower((rRPower / max) * speed);

        // === AUTO-LOCK MODE ===
        if (autoLockActive) {
            // Auto-lock is ACTIVE - track AprilTag
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();

                if (Math.abs(tx) > TOLERANCE) {
                    // Not aligned - adjust turret servo
                    double adjustment = calculateServoAdjustment(tx, deltaTime);
                    targetPosition = currentPosition + adjustment;

                    // Handle wrapping for continuous rotation
                    // The 5-turn servo uses 0.0-1.0 range, but for 360° you may not use the full range
                    // depending on your gear ratio. Wrapping ensures smooth continuous rotation.
                    if (targetPosition > SERVO_MAX) {
                        targetPosition = SERVO_MIN + (targetPosition - SERVO_MAX);
                    } else if (targetPosition < SERVO_MIN) {
                        targetPosition = SERVO_MAX - (SERVO_MIN - targetPosition);
                    }

                    targetPosition = clamp(targetPosition, SERVO_MIN, SERVO_MAX);

                    // Smooth interpolation
                    double smoothFactor = SMOOTHNESS * (deltaTime * 60.0);
                    smoothFactor = Math.min(smoothFactor, 1.0);
                    currentPosition = currentPosition + (targetPosition - currentPosition) * smoothFactor;

                    // Apply servo position
                    turretServo.setPosition(currentPosition);

                    // Telemetry
                    telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
                    telemetry.addLine(tx > 0 ? ">>> TRACKING RIGHT >>>" : "<<< TRACKING LEFT <<<");
                    telemetry.addData("Error", "%.2f°", tx);
                    telemetry.addData("Servo Position", "%.3f", currentPosition);
                    telemetry.addData("Adjustment", "%.4f", adjustment);

                } else {
                    // Aligned!
                    targetPosition = currentPosition;
                    turretServo.setPosition(currentPosition);

                    telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
                    gamepad1.rumble(100); // Vibrate (duration in milliseconds)
                    telemetry.addLine("✓✓✓ LOCKED ON TARGET ✓✓✓");
                    telemetry.addData("Error", "%.2f°", tx);
                    telemetry.addData("Servo Position", "%.3f", currentPosition);
                }

            } else {
                // No target - hold current position
                targetPosition = currentPosition;
                turretServo.setPosition(currentPosition);

                telemetry.addLine("🔒 AUTO-LOCK ACTIVE 🔒");
                telemetry.addLine("⚠ NO TARGET DETECTED ⚠");
                telemetry.addData("Servo Position", "%.3f", currentPosition);
            }

        } else {
            // Auto-lock is OFF - manual turret control with D-pad
            // Note: Right stick X is used for robot rotation in mecanum drive
            // Use D-pad for manual turret adjustment
            double manualControl = 0;
            if (gamepad1.dpad_left) {
                manualControl = -0.01; // Rotate turret left
            } else if (gamepad1.dpad_right) {
                manualControl = 0.01;  // Rotate turret right
            }

            currentPosition = clamp(currentPosition + manualControl, SERVO_MIN, SERVO_MAX);
            targetPosition = currentPosition;
            turretServo.setPosition(currentPosition);

            telemetry.addLine("Manual Control Mode");
            telemetry.addLine("Press R3 to enable Auto-Lock");
            telemetry.addLine("Use D-pad Left/Right for manual turret");
            telemetry.addData("Servo Position", "%.3f", currentPosition);
            telemetry.addData("Manual Input", "%.3f", manualControl);
        }

        // Status display
        telemetry.addLine();
        telemetry.addData("Auto-Lock Status", autoLockActive ? "🔒 ACTIVE" : "OFF");
        telemetry.addData("FPS", "%.1f", 1.0 / Math.max(deltaTime, 0.001));
        telemetry.update();
    }

    private double calculateServoAdjustment(double error, double deltaTime) {
        // Normalize error (assuming max useful error is around 30 degrees)
        double normalizedError = Math.abs(error) / 30.0;
        normalizedError = Math.min(normalizedError, 1.0);

        // Calculate adjustment magnitude with mixed linear/squared response
        double linearComponent = normalizedError * 0.4;
        double squaredComponent = normalizedError * normalizedError * 0.6;
        double magnitude = (linearComponent + squaredComponent) * MAX_SERVO_SPEED;

        // Apply minimum speed threshold
        if (magnitude > 0 && magnitude < MIN_SERVO_SPEED) {
            magnitude = MIN_SERVO_SPEED;
        }

        // Scale by delta time for frame-rate independence
        magnitude *= (deltaTime * 60.0);

        // Return signed adjustment
        // Limelight tx: positive = target is to the right, negative = target is to the left
        // If REVERSE_DIRECTION is true, flip the sign because gears reverse the motion
        double adjustment = Math.signum(error) * magnitude;

        if (REVERSE_DIRECTION) {
            adjustment = -adjustment;
        }

        return adjustment;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        turretServo.setPosition(SERVO_CENTER);
        limelight.stop();
    }
}