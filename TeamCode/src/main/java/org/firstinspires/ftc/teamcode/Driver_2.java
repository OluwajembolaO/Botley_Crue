package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Driver2")
public class Driver_2 extends OpMode {
    // Drive motors for gamepad 1
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    // Intake/outtake motors for gamepad 2
    private DcMotor intake;
    private DcMotor outtake1;
    private DcMotor outtake2;

    // Outtake speed stuff - starts at 50% so we don't blow anything up lol
    private double outtakeSpeed = 0.5;
    private final double SPEED_INCREMENT = 0.1; // goes up/down by 10% each press
    private final double MIN_SPEED = 0.3; // don't go below 30% or it's kinda useless
    private final double MAX_SPEED = 1.0; // full send mode

    // These stop you from spamming buttons and breaking everything
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void init() {
        // grab all the motors from config
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        rearRightMotor = hardwareMap.get(DcMotor.class, "motor4");

        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");

        // right side motors go backwards normally so we flip em
        topRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // make one outtake spin the other way so they work together
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Outtake Speed", "%.0f%%", outtakeSpeed * 100);
        telemetry.update();
    }

    @Override
    public void loop() {
        // ========== GAMEPAD 1: DRIVING STUFF ==========
        double speed = 0.65; // normal speed
        double fwd = -gamepad1.left_stick_x; // forward/back
        double str = gamepad1.left_stick_y;  // side to side
        double rot = -gamepad1.right_stick_x; // spinning

        float lt = gamepad1.left_trigger;   // slow mode
        float rt = gamepad1.right_trigger;  // zoom mode

        // triggers change how fast we go
        if(lt > 0.5) {
            speed = 0.4; // chill mode
        }
        if(rt > 0.5) {
            speed = 0.9; // gotta go fast
        }

        // mecanum drive math (don't touch this unless you know what you're doing)
        double tLPower = fwd + str + rot;
        double rLPower = fwd - str - rot;
        double tRPower = fwd - str + rot;
        double rRPower = fwd + str - rot;

        // makes sure motors don't try to go over 100%
        double max = Math.max(1.0, Math.abs(tLPower));
        max = Math.max(max, Math.abs(rLPower));
        max = Math.max(max, Math.abs(tRPower));
        max = Math.max(max, Math.abs(rRPower));

        // actually make the motors go
        topLeftMotor.setPower((tLPower / max) * speed);
        topRightMotor.setPower((tRPower / max) * speed);
        rearLeftMotor.setPower((rLPower / max) * speed);
        rearRightMotor.setPower((rRPower / max) * speed);

        // ========== GAMEPAD 2: INTAKE/OUTTAKE CONTROLS ==========

        // left bumper = intake go brrr
        if (gamepad2.left_bumper) {
            intake.setPower(1.0);
        } else if (gamepad2.left_trigger > 0.5) {
            // left trigger = oops wrong way
            intake.setPower(-1.0);
        } else {
            intake.setPower(0);
        }

        // dpad up/down = make outtake faster/slower (just in case you guys are slow)
        if (gamepad2.dpad_up && !dpadUpPressed) {
            outtakeSpeed += SPEED_INCREMENT;
            if (outtakeSpeed > MAX_SPEED) {
                outtakeSpeed = MAX_SPEED; // can't go above 100%
            }
            dpadUpPressed = true;
        } else if (!gamepad2.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad2.dpad_down && !dpadDownPressed) {
            outtakeSpeed -= SPEED_INCREMENT;
            if (outtakeSpeed < MIN_SPEED) {
                outtakeSpeed = MIN_SPEED; // don't go too slow
            }
            dpadDownPressed = true;
        } else if (!gamepad2.dpad_down) {
            dpadDownPressed = false;
        }

        // right bumper = outtake go brrr at whatever speed we set
        if (gamepad2.right_bumper) {
            outtake1.setPower(outtakeSpeed);
            outtake2.setPower(outtakeSpeed);
        } else if (gamepad2.right_trigger > 0.5) {
            // right trigger = reverse outtake if you messed up
            outtake1.setPower(-outtakeSpeed);
            outtake2.setPower(-outtakeSpeed);
        } else {
            outtake1.setPower(0);
            outtake2.setPower(0);
        }

        // shows stuff on the driver station so you know what's happening
        telemetry.addData("Drive Speed", "%.0f%%", speed * 100);
        telemetry.addData("Outtake Speed", "%.0f%%", outtakeSpeed * 100);
        telemetry.addData("Intake Active", gamepad2.left_bumper);
        telemetry.addData("Outtake Active", gamepad2.right_bumper);
        telemetry.update();
    }
}