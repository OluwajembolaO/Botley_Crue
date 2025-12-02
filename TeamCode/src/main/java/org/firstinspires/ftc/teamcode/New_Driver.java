package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="NewDriver")
public class New_Driver extends OpMode {
    // Motors
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    private DcMotor intake;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;

    // Speed constants to tune easily, i get lazy :3
    private static final double NORMAL_SPEED = 0.65;
    private static final double SLOW_SPEED = 0.4;
    private static final double FAST_SPEED = 0.9;
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double ROTATION_MULTIPLIER = 0.7; // Adjust iff rotation feels off

    // Intake/Outtake constantssss :)
    private static final double INTAKE_POWER = 1.0;

    private  static  final double TICKS_PER_REV = 6000.0;
    private static final double GEAR_RATIO = 1.0;

    //Change from final
    private double targetRPM = 4000;
    private static final double RPM_INCREMENT = 100;
    private static final double MAX_RPM = 6000;
    private static final double MIN_RPM = 0;

    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;


    private static final double TARGET_RPM = 4000;
    private static final double TARGET_TICKS_PER_SEC = (TARGET_RPM * TICKS_PER_REV * GEAR_RATIO) / 60.0;
//    private static final double OUTTAKE_POWER = .75;

    @Override
    public void init() {
        // Initialize motors
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        rearRightMotor = hardwareMap.get(DcMotor.class, "motor4");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        // Set motor directions
        topRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        //enable built-in velocity PID on outtake motors
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Float for flywheels (shoting)
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // BRAKE mode - robot stops instantly instead of coasting
        topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Target:" + TARGET_RPM + " RPM");
    }

    @Override
    public void loop() {
        // Handle intake/outtake, i learned what a ? means .-.
        intake.setPower(gamepad1.left_bumper ? INTAKE_POWER : 0.0);


        if (gamepad1.dpad_up && !dpadUpPressed){
            targetRPM = Math.min(targetRPM + RPM_INCREMENT, MAX_RPM);
        }
        if (gamepad1.dpad_down && !dpadDownPressed){
            targetRPM = Math.max(targetRPM - RPM_INCREMENT, MIN_RPM);
        }

        dpadUpPressed = gamepad1.dpad_up;
        dpadDownPressed = gamepad1.dpad_down;


        double targetTicksPerSec = (targetRPM * TICKS_PER_REV * GEAR_RATIO) / 60.0;

        if (gamepad1.right_bumper){
            outtake1.setVelocity(targetTicksPerSec);
            outtake2.setVelocity(targetTicksPerSec);
        } else{
            outtake1.setVelocity(0);
            outtake2.setVelocity(0);
        }



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


        double rpm1 = (outtake1.getVelocity() / TICKS_PER_REV / GEAR_RATIO) * 60;
        double rpm2 = (outtake2.getVelocity() / TICKS_PER_REV / GEAR_RATIO) * 60;

        telemetry.addData("Target", "%.0f RPM (D-pad to adjust)", TARGET_RPM);
        telemetry.addData("Actual", "%.0f / %.0f RPM", rpm1, rpm2);
        telemetry.addData("Shooter", gamepad1.right_bumper ? "SPINNING" : "OFF");
        telemetry.update();

    }
}