package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="NewDriver")
public class New_Driver extends OpMode {
    // Motors
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    private DcMotor intake;
    private DcMotor outtake1;
    private DcMotor outtake2;

    // Speed constants to tune easily, i get lazy :3
    private static final double NORMAL_SPEED = 0.65;
    private static final double SLOW_SPEED = 0.4;
    private static final double FAST_SPEED = 0.9;
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double ROTATION_MULTIPLIER = 0.7; // Adjust iff rotation feels off

    // Intake/Outtake constantssss :)
    private static final double INTAKE_POWER = 1.0;
    private static final double OUTTAKE_POWER = .75;

    @Override
    public void init() {
        // Initialize motors
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        rearRightMotor = hardwareMap.get(DcMotor.class, "motor4");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");

        // Set motor directions
        topRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        // BRAKE mode - robot stops instantly instead of coasting
        topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // Handle intake/outtake, i learned what a ? means .-.
        intake.setPower(gamepad1.left_bumper ? INTAKE_POWER : 0.0);
        double outtakePower = gamepad1.right_bumper ? OUTTAKE_POWER : 0.0;
        outtake1.setPower(outtakePower);
        outtake2.setPower(outtakePower);

        // Get controller inputs
        double fwd = -gamepad1.left_stick_x;  // Your strafe
        double str = gamepad1.left_stick_y;   // Your forward/back
        double rot = -gamepad1.right_stick_x * ROTATION_MULTIPLIER;

        // Determine speed based on triggers
        double speed = NORMAL_SPEED;
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            speed = SLOW_SPEED;
        }
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            speed = FAST_SPEED;
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

        // Set motor powers , mathy  math
        topLeftMotor.setPower(tLPower * speed);
        topRightMotor.setPower(tRPower * speed);
        rearLeftMotor.setPower(rLPower * speed);
        rearRightMotor.setPower(rRPower * speed);
    }
}