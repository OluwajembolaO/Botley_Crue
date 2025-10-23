package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="AprilTagAutoAlign")
public class AprilTagAutoAlign extends OpMode {

    // Motors for mecanum drive
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    // Sensors
    private Limelight3A limelight;
    private IMU imu;

    // Simple bang-bang control parameters
    private static final double DEADZONE = 5.0;          // degrees - if TX is within this, stop turning
    private static final double TURN_POWER = 0.3;        // fixed turning speed when aligning

    private boolean autoAlignEnabled = false;
    private boolean xButtonWasPressed = false;  // Track previous button state

    @Override
    public void init() {
        // Motor initialization
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        rearRightMotor = hardwareMap.get(DcMotor.class, "motor4");

        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sensor initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(RevOrientation));

        telemetry.addLine("Press X to toggle auto-align");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        // Toggle auto-align with X button
        if (gamepad1.x) {
            autoAlignEnabled = !autoAlignEnabled;
            while (gamepad1.x) {
                // Wait for button release
            }
        }

        // Get robot IMU yaw and update limelight
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        // Get Limelight result
        LLResult llResult = limelight.getLatestResult();

        // Default joystick inputs
        double fwd = -gamepad1.left_stick_y;  // forward/backward
        double str = gamepad1.left_stick_x;   // strafe left/right
        double rot = gamepad1.right_stick_x;  // rotation

        // Auto-align logic - SIMPLE BANG-BANG CONTROL
        if (autoAlignEnabled && llResult != null && llResult.isValid()) {

            // Get TX - horizontal angle to target
            // Negative TX = tag is LEFT of center
            // Positive TX = tag is RIGHT of center
            double tx = llResult.getTx();

            telemetry.addData("TX (angle to tag)", "%.2f deg", tx);

            // Simple decision: which way to turn?
            if (Math.abs(tx) > DEADZONE) {
                // Tag is NOT centered - need to turn
                if (tx > 0) {
                    // Tag is to the RIGHT - turn RIGHT
                    rot = TURN_POWER;
                    telemetry.addLine("Turning RIGHT →");
                } else {
                    // Tag is to the LEFT - turn LEFT
                    rot = -TURN_POWER;
                    telemetry.addLine("Turning LEFT ←");
                }
            } else {
                // Tag is centered enough - STOP turning
                rot = 0;
                telemetry.addLine("✓✓✓ LOCKED ON ✓✓✓");
            }

            // Allow manual forward/back and strafe during auto-align
            fwd = -gamepad1.left_stick_y;
            str = gamepad1.left_stick_x;

        } else if (autoAlignEnabled) {
            telemetry.addLine("⚠ No AprilTag detected");
            telemetry.addLine("Manual control active");
        }

        // Calculate mecanum motor powers
        double tLPower = fwd + str + rot;
        double rLPower = fwd - str - rot;
        double tRPower = fwd - str + rot;
        double rRPower = fwd + str - rot;

        // Normalize motor powers ONLY if any exceed 1.0
        double max = Math.max(Math.abs(tLPower), Math.abs(rLPower));
        max = Math.max(max, Math.abs(tRPower));
        max = Math.max(max, Math.abs(rRPower));

        if (max > 1.0) {
            topLeftMotor.setPower(tLPower / max);
            topRightMotor.setPower(tRPower / max);
            rearLeftMotor.setPower(rLPower / max);
            rearRightMotor.setPower(rRPower / max);
        } else {
            topLeftMotor.setPower(tLPower);
            topRightMotor.setPower(tRPower);
            rearLeftMotor.setPower(rLPower);
            rearRightMotor.setPower(rRPower);
        }

        // Telemetry
        telemetry.addData("Auto-Align", autoAlignEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("FWD", "%.2f", fwd);
        telemetry.addData("STR", "%.2f", str);
        telemetry.addData("ROT", "%.2f", rot);
        telemetry.update();
    }
}