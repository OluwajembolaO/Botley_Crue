package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Driver1")
public class Driver_1 extends OpMode {
    // the 4 drive motors and our shooting stuff - pls no more motor controls ;-;
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    private DcMotor intake;
    private DcMotor outtake1;   // first outtake motor
    private DcMotor outtake2;   // second outtake motor

    @Override
    public void init() {
        // grab all the motors from the config
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        rearRightMotor = hardwareMap.get(DcMotor.class, "motor4");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");

        topRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);

        // ADD THESE - huge improvement for competition!
        topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void loop() {
        // left bumper = intake go
        intake.setPower(0);
        if (gamepad1.left_bumper){
            intake.setPower(1);
        }

        // right bumper = outtake go brrr
        // both motors spin at same power but opposite directions cause we reversed one
        outtake1.setPower(0);
        outtake2.setPower(0);
        if (gamepad1.right_bumper){
            outtake1.setPower(1);
            outtake2.setPower(1);  // spins opposite cause of setDirection above
        }

        double speed = 0.65; // normal driving speed
        double fwd = -gamepad1.left_stick_x; // forward/back - using X axis idk dawg
        double str = gamepad1.left_stick_y;  // side to side (strafe) - using Y axis
        double rot = -gamepad1.right_stick_x; // spinning around - using the formula that actually works

        float lt = gamepad1.left_trigger;   // slow mode trigger
        float rt = gamepad1.right_trigger;  // fast mode trigger

        // triggers make you go slower or faster
        if(lt > 0.5) {
            speed = 0.4; // chill mode activated
        }
        if(rt > 0.5) {
            speed = 0.9; // zoom zoom mode
        }

        // mecanum drive math (seriously don't mess with this)
        double tLPower = fwd + str + rot;
        double rLPower = fwd - str - rot;
        double tRPower = fwd - str + rot;
        double rRPower = fwd + str - rot;

        // makes sure motors don't try to go over 100% :> YIPEEEEE
        double max = Math.max(1.0, Math.abs(tLPower));
        max = Math.max(max, Math.abs(rLPower));
        max = Math.max(max, Math.abs(tRPower));
        max = Math.max(max, Math.abs(rRPower));

        // actually send power to the motors .-. (too much math imo)
        topLeftMotor.setPower((tLPower / max) * speed);
        topRightMotor.setPower((tRPower / max) * speed);
        rearLeftMotor.setPower((rLPower / max) * speed);
        rearRightMotor.setPower((rRPower / max) * speed);
    }
}