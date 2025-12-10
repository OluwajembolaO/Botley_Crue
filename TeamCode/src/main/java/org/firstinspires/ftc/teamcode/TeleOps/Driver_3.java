package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Driver3_BLUE")
public class Driver_3 extends OpMode {
    // Drive motors for gamepad 1
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    // Intake/outtake/transfer motors for gamepad 2 hi
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;

    // Limelight for autolock
    private Limelight3A limelight;

    // Speed constants - easy to tune
    private static final double NORMAL_SPEED = 0.65;
    private static final double SLOW_SPEED = 0.4;
    private static final double FAST_SPEED = 0.9;
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double ROTATION_MULTIPLIER = 0.7;

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
    private static final double RPM_TOLERANCE = 200; // RPM within this range = at target

    // Autolock constants
    private static final double AUTOLOCK_KP = 0.03; // Proportional gain for rotation correction
    private static final double AUTOLOCK_TOLERANCE = 3.0; // degrees
    private static final double MAX_AUTOLOCK_ROTATION = 0.5; // Max rotation speed during autolock
    private static final double MIN_AUTOLOCK_ROTATION = 0.1; // Min rotation speed to overcome friction

    // Button state tracking
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean autoLockActive = false;
    private boolean gp1LeftBumperWasPressed = false;

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


        telemetry.addLine("Driver2 Initialized");
        telemetry.addLine("Gamepad 1: Drive + Autolock (LB)");
        telemetry.addLine("Gamepad 2: Intake/Transfer/Outtake");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.pipelineSwitch(9  );
        limelight.start();
        timer.reset();
    }

    @Override
    public void loop() {
        // ========== AUTOLOCK TOGGLE (Gamepad 1 Left Bumper) ==========

        // Toggle autolock with gamepad1 left bumper
        if (gamepad1.left_bumper && !gp1LeftBumperWasPressed) {
            autoLockActive = !autoLockActive;
        }
        gp1LeftBumperWasPressed = gamepad1.left_bumper;

        // ========== GAMEPAD 1: DRIVING ==========

        // Get controller inputs for driving
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
                double tx = llResult.getTx(); // Horizontal offset in degrees

                if (Math.abs(tx) > AUTOLOCK_TOLERANCE) {
                    // Calculate rotation correction using proportional control
                    double rotationCorrection = tx * AUTOLOCK_KP;

                    // Clamp rotation correction
                    rotationCorrection = Math.max(-MAX_AUTOLOCK_ROTATION,
                            Math.min(MAX_AUTOLOCK_ROTATION, rotationCorrection));

                    // Apply minimum rotation speed to overcome friction
                    if (Math.abs(rotationCorrection) > 0 && Math.abs(rotationCorrection) < MIN_AUTOLOCK_ROTATION) {
                        rotationCorrection = Math.signum(rotationCorrection) * MIN_AUTOLOCK_ROTATION;
                    }

                    // Override manual rotation with autolock correction
                    rot = -rotationCorrection; // Negative because tx positive means turn right

                    telemetry.addLine("🔒 AUTOLOCK ACTIVE - TRACKING 🔒");
                    telemetry.addData("Target Error", "%.2f°", tx);
                    telemetry.addData("Rotation Correction", "%.3f", rotationCorrection);
                } else {
                    // Locked on target
                    rot = 0; // Stop rotation
                    gamepad1.rumble(100); // Vibrate gamepad1 when locked
                    telemetry.addLine("🔒 AUTOLOCK ACTIVE - LOCKED ✓✓✓");
                    telemetry.addData("Target Error", "%.2f°", tx);
                }
            } else {
                // No target detected
                telemetry.addLine("🔒 AUTOLOCK ACTIVE - NO TARGET ⚠");
                telemetry.addLine("Manual control available");
            }
        }

        // Mecanum drive calculations
        double tLPower = fwd + str + rot;
        double rLPower = fwd - str - rot;
        double tRPower = fwd - str + rot;
        double rRPower = fwd + str - rot;

        // Normalize powers if any exceed 1.0
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

        // ========== GAMEPAD 2: INTAKE/TRANSFER/OUTTAKE CONTROLS ==========

        // Intake control - Left bumper hold-type (only runs while held)
        if (gamepad2.left_bumper) {
            intake.setPower(INTAKE_POWER);
        } else if (gamepad2.x) {  // Square button (X on Xbox) - reverse
            intake.setPower(-INTAKE_POWER);
        } else {
            intake.setPower(0);
        }

        // Transfer control - Right trigger (forward) and X button (reverse)
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            transfer.setPower(TRANSFER_POWER);
        } else if (gamepad2.a) {  // X button - reverse transfer
            transfer.setPower(-TRANSFER_POWER);
        } else {
            transfer.setPower(0);
        }

        // RPM adjustment with dpad
        if (gamepad2.dpad_up && !dpadUpPressed) {
            targetRPM = Math.min(targetRPM + RPM_INCREMENT, MAX_RPM);
            dpadUpPressed = true;
        } else if (!gamepad2.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad2.dpad_down && !dpadDownPressed) {
            targetRPM = Math.max(targetRPM - RPM_INCREMENT, MIN_RPM);
            dpadDownPressed = true;
        } else if (!gamepad2.dpad_down) {
            dpadDownPressed = false;
        }

        // Calculate target velocity in ticks per second
        double targetTicksPerSec = (targetRPM * TICKS_PER_REV * GEAR_RATIO) / 60.0;

        // Outtake control - Right bumper (forward) and Y button (reverse)
        if (gamepad2.right_bumper) {
            outtake1.setVelocity(targetTicksPerSec);
            outtake2.setVelocity(targetTicksPerSec);
        } else if (gamepad2.y) {  // Y button - reverse outtake
            outtake1.setVelocity(-targetTicksPerSec);
            outtake2.setVelocity(-targetTicksPerSec);
        } else {
            outtake1.setVelocity(0);
            outtake2.setVelocity(0);
        }

        // Calculate actual RPM for telemetry
        double rpm1 = (outtake1.getVelocity() / TICKS_PER_REV / GEAR_RATIO) * 60;
        double rpm2 = (outtake2.getVelocity() / TICKS_PER_REV / GEAR_RATIO) * 60;

        // Check if outtake is at target RPM and vibrate gamepad2
        boolean atTargetRPM = false;
        if (gamepad2.right_bumper && targetRPM > 0) {
            double avgRPM = (rpm1 + rpm2) / 2.0;
            if (Math.abs(avgRPM - targetRPM) <= RPM_TOLERANCE) {
                atTargetRPM = true;
                gamepad2.rumble(50);  // Vibrate gamepad2 when at target RPM
            }
        }

        // Telemetry
        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Speed Mode", speed == SLOW_SPEED ? "SLOW" : speed == FAST_SPEED ? "FAST" : "NORMAL");
        telemetry.addData("Autolock", autoLockActive ? "🔒 ON (GP1 LB to toggle)" : "OFF (GP1 LB to toggle)");
        telemetry.addLine();
        telemetry.addLine("=== INTAKE/TRANSFER/OUTTAKE ===");
        telemetry.addData("Intake", gamepad2.left_bumper ? "ON (HOLD)" : gamepad2.x ? "REVERSE" : "OFF");
        telemetry.addData("Transfer", gamepad2.right_trigger > TRIGGER_THRESHOLD ? "FORWARD" : gamepad2.x ? "REVERSE" : "OFF");
        telemetry.addData("Target RPM", "%.0f (D-pad to adjust)", targetRPM);
        telemetry.addData("Actual RPM", "%.0f / %.0f", rpm1, rpm2);
        telemetry.addData("Outtake", gamepad2.right_bumper ? "FORWARD" : gamepad2.y ? "REVERSE" : "OFF");
        telemetry.addData("RPM Status", atTargetRPM ? "✓ AT TARGET" : "...");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
