package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Driver1")
public class Driver_1 extends OpMode {
    //Represents the 4 motors I know we are probably using like 6-8 motors but this is just the basics
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    private DcMotor intake;

    @Override
    public void init() {
        //Sets the motors
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        rearRightMotor = hardwareMap.get(DcMotor.class, "motor4");
        intake = hardwareMap.get(DcMotor.class, "intake");

        //Why the reverse? Left side move opposite way from right so I would have to reverse it
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        intake.setPower(0);
        if (gamepad1.left_bumper){
            intake.setPower(1);
        }
        double speed = 0.65; // Default 50% power
        double fwd = gamepad1.left_stick_y; // forward/back(Y axis) - REMOVED negative sign
        double str = -gamepad1.left_stick_x;  // left/right(Strife/X axis)
        double rot = gamepad1.right_stick_x; // rotation from right stick(Spin go burrrrr)

        float lt = gamepad1.left_trigger;   // Slow down trigger
        float rt = gamepad1.right_trigger;  // Speed up trigger

        // Adjust speed based on triggers (0.0 to 1.0)
        if(lt > 0.5) {
            speed = 0.4; // 25% power when left trigger pressed
        }
        if(rt > 0.5) {
            speed = 0.9; // 75% power when right trigger pressed
        }

        double tLPower = fwd + str + rot;
        double rLPower = fwd - str + rot;
        double tRPower = fwd - str - rot;
        double rRPower = fwd + str - rot;

        // Normalize the motor powers
        double max = Math.max(1.0, Math.abs(tLPower));
        max = Math.max(max, Math.abs(rLPower));
        max = Math.max(max, Math.abs(tRPower));
        max = Math.max(max, Math.abs(rRPower));

        // Apply normalized power multiplied by speed
        topLeftMotor.setPower((tLPower / max) * speed);
        topRightMotor.setPower((tRPower / max) * speed);
        rearLeftMotor.setPower((rLPower / max) * speed);
        rearRightMotor.setPower((rRPower / max) * speed);
    }
}