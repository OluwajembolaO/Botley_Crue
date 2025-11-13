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
    private DcMotor outtake1;   // Changed from single outtake
    private DcMotor outtake2;  // Added second outtake motor

    @Override
    public void init() {
        //Sets the motors
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        rearRightMotor = hardwareMap.get(DcMotor.class, "motor4");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");

        //Why the reverse? Left side move opposite way from right so I would have to reverse it
        topRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse one outtake motor so they spin opposite directions
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // Intake control - me use left bumper
        intake.setPower(0);
        if (gamepad1.left_bumper){
            intake.setPower(1);
        }

        // Outtake control - me use right bumper
        // Both motors run at same power but opposite directions
        outtake1.setPower(0);
        outtake2.setPower(0);
        if (gamepad1.right_bumper){
            outtake1.setPower(1);
            outtake2.setPower(1);  // Will spin opposite due to setDirection
        }

        double speed = 0.65; // Default 50% power
        double fwd = -gamepad1.left_stick_x; // forward/back - now using X axis idk dawg
        double str = gamepad1.left_stick_y;  // left/right(Strafe) - now using Y axis
        double rot = -gamepad1.right_stick_x; // rotation from right stick - using original working formula

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
        double rLPower = fwd - str - rot;
        double tRPower = fwd - str + rot;
        double rRPower = fwd + str - rot;

        // Normalize the motor powers :> YIPEEEEE
        double max = Math.max(1.0, Math.abs(tLPower));
        max = Math.max(max, Math.abs(rLPower));
        max = Math.max(max, Math.abs(tRPower));
        max = Math.max(max, Math.abs(rRPower));

        // Apply normalized power multiplied by speed .-. (too much math imo)
        topLeftMotor.setPower((tLPower / max) * speed);
        topRightMotor.setPower((tRPower / max) * speed);
        rearLeftMotor.setPower((rLPower / max) * speed);
        rearRightMotor.setPower((rRPower / max) * speed);
    }
}