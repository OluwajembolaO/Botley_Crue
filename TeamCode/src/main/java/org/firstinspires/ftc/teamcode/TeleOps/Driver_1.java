package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Driver_1")
public class Driver_1 extends OpMode {
    // Drive motors
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    // Intake/outtake/transfer motors
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;

    // Limelight for autolock
    private Limelight3A limelight;

    // Speed constants
    private static final double NORMAL_SPEED = 0.9;
    private static final double SLOW_SPEED = 0.8;
    private static final double FAST_SPEED = 1;
    private static final double TRIGGER_THRESHOLD = 0.5;

    // Intake/Transfer constants
    private static final double INTAKE_POWER = 0.5;
    private static final double TRANSFER_POWER = 0.9;

    // Outtake RPM control constants
    private static final double TICKS_PER_REV = 6000.0;
    private static final double GEAR_RATIO = 1.0;
    private double targetRPM = 4000;
    private static final double RPM_INCREMENT = 100;
    private static final double MAX_RPM = 6000;
    private static final double MIN_RPM = 0;
    private static final double RPM_TOLERANCE = 200;

    // Autolock constants
    private static final double AUTOLOCK_KP = 0.03;
    private static final double AUTOLOCK_TOLERANCE = 3.0;
    private static final double MAX_AUTOLOCK_ROTATION = 0.5;
    private static final double MIN_AUTOLOCK_ROTATION = 0.1;

    // Button state tracking
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean autoLockActive = false;
    private boolean yWasPressed = false;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        // Initialize drive motors
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        rearRightMotor = hardwareMap.get(DcMotor.class, "motor4");

        // Initialize intake/transfer/outtake motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set motor directions
        topRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Enable built-in velocity PID on outtake motors
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Float for flywheels (shooting)
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        telemetry.addLine("=== SINGLE GAMEPAD CONTROLS ===");
        telemetry.addLine();
        telemetry.addLine("DRIVING:");
        telemetry.addLine("  Left Stick: Strafe/Forward");
        telemetry.addLine("  Right Stick X: Rotate");
        telemetry.addLine("  Left Trigger: Slow mode");
        telemetry.addLine("  Right Trigger: Fast mode");
        telemetry.addLine();
        telemetry.addLine("MECHANISMS:");
        telemetry.addLine("  Left Bumper: Intake forward");
        telemetry.addLine("  X (Square): Intake reverse");
        telemetry.addLine("  A (Cross): Transfer");
        telemetry.addLine("  Right Bumper: Outtake spin");
        telemetry.addLine("  D-Pad Up/Down: Adjust RPM");
        telemetry.addLine("  Y (Triangle): Toggle Autolock");
        telemetry.addLine();
        telemetry.addData("Target RPM", targetRPM);
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
        // ========== AUTOLOCK TOGGLE (Y button) ==========
        if (gamepad1.y && !yWasPressed) {
            autoLockActive = !autoLockActive;
        }
        yWasPressed = gamepad1.y;

        // ========== DRIVING ==========
        double fwd = -gamepad1.left_stick_x;  // strafe
        double str = gamepad1.left_stick_y;   // forward/back
        double rot = -gamepad1.right_stick_x;

        // Determine speed based on triggers
        double speed = NORMAL_SPEED;
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            speed = SLOW_SPEED;
        }
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            speed = FAST_SPEED;
        }

        // Apply autolock rotation correction if active
        if (autoLockActive) {
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();

                if (Math.abs(tx) > AUTOLOCK_TOLERANCE) {
                    double rotationCorrection = tx * AUTOLOCK_KP;

                    rotationCorrection = Math.max(-MAX_AUTOLOCK_ROTATION,
                            Math.min(MAX_AUTOLOCK_ROTATION, rotationCorrection));

                    if (Math.abs(rotationCorrection) > 0 && Math.abs(rotationCorrection) < MIN_AUTOLOCK_ROTATION) {
                        rotationCorrection = Math.signum(rotationCorrection) * MIN_AUTOLOCK_ROTATION;
                    }

                    rot = -rotationCorrection;

                    telemetry.addLine("🔒 AUTOLOCK ACTIVE - TRACKING 🔒");
                    telemetry.addData("Target Error", "%.2f°", tx);
                    telemetry.addData("Rotation Correction", "%.3f", rotationCorrection);
                } else {
                    rot = 0;
                    gamepad1.rumble(100);
                    telemetry.addLine("🔒 AUTOLOCK ACTIVE - LOCKED ✓✓✓");
                    telemetry.addData("Target Error", "%.2f°", tx);
                }
            } else {
                telemetry.addLine("🔒 AUTOLOCK ACTIVE - NO TARGET ⚠");
                telemetry.addLine("Manual control available");
            }
        }

        // Mecanum drive calculations
        double tLPower = fwd + str + rot;
        double rLPower = fwd - str - rot;
        double tRPower = fwd - str + rot;
        double rRPower = fwd + str - rot;

        // Normalize powers
        double max = Math.max(Math.abs(tLPower), Math.abs(rLPower));
        max = Math.max(max, Math.abs(tRPower));
        max = Math.max(max, Math.abs(rRPower));

        if (max > 1.0) {
            tLPower /= max;
            rLPower /= max;
            tRPower /= max;
            rRPower /= max;
        }

        // Set motor powers
        topLeftMotor.setPower(tLPower * speed);
        topRightMotor.setPower(tRPower * speed);
        rearLeftMotor.setPower(rLPower * speed);
        rearRightMotor.setPower(rRPower * speed);

        // ========== INTAKE/TRANSFER/OUTTAKE CONTROLS ==========

        // Intake control - Left bumper (forward) and X (reverse)
        if (gamepad1.left_bumper) {
            intake.setPower(INTAKE_POWER);
        } else if (gamepad1.x) {
            intake.setPower(-INTAKE_POWER);
        } else {
            intake.setPower(0);
        }

        // Transfer control - A button
        if (gamepad1.a) {
            transfer.setPower(TRANSFER_POWER);
        } else {
            transfer.setPower(0);
        }

        // RPM adjustment with dpad
        if (gamepad1.dpad_up && !dpadUpPressed) {
            targetRPM = Math.min(targetRPM + RPM_INCREMENT, MAX_RPM);
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            targetRPM = Math.max(targetRPM - RPM_INCREMENT, MIN_RPM);
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // Calculate target velocity
        double targetTicksPerSec = (targetRPM * TICKS_PER_REV * GEAR_RATIO) / 60.0;

        // Outtake control - Right bumper
        if (gamepad1.right_bumper) {
            outtake1.setVelocity(targetTicksPerSec);
            outtake2.setVelocity(targetTicksPerSec);
        } else {
            outtake1.setVelocity(0);
            outtake2.setVelocity(0);
        }

        // Calculate actual RPM
        double rpm1 = (outtake1.getVelocity() / TICKS_PER_REV / GEAR_RATIO) * 60;
        double rpm2 = (outtake2.getVelocity() / TICKS_PER_REV / GEAR_RATIO) * 60;

        // Check if outtake is at target RPM
        boolean atTargetRPM = false;
        if (gamepad1.right_bumper && targetRPM > 0) {
            double avgRPM = (rpm1 + rpm2) / 2.0;
            if (Math.abs(avgRPM - targetRPM) <= RPM_TOLERANCE) {
                atTargetRPM = true;
                gamepad1.rumble(50);
            }
        }

        // Telemetry
        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Speed Mode", speed == SLOW_SPEED ? "SLOW" : speed == FAST_SPEED ? "FAST" : "NORMAL");
        telemetry.addData("Autolock", autoLockActive ? "🔒 ON (Y to toggle)" : "OFF (Y to toggle)");
        telemetry.addLine();
        telemetry.addLine("=== MECHANISMS ===");
        telemetry.addData("Intake", gamepad1.left_bumper ? "FORWARD" : gamepad1.x ? "REVERSE" : "OFF");
        telemetry.addData("Transfer", gamepad1.a ? "RUNNING" : "OFF");
        telemetry.addData("Target RPM", "%.0f (D-pad to adjust)", targetRPM);
        telemetry.addData("Actual RPM", "%.0f / %.0f", rpm1, rpm2);
        telemetry.addData("Outtake", gamepad1.right_bumper ? "SPINNING" : "OFF");
        telemetry.addData("RPM Status", atTargetRPM ? "✓ AT TARGET" : "...");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}