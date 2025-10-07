package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="VideoNotes")
public class VideoNotes extends OpMode {
    //Represents the 4 motors I know we are probably using like 6-8 motors but this is just the basics
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    @Override
    public void init() {
        //Sets the motors
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        rearRightMotor = hardwareMap.get(DcMotor.class, "motor4");

        //Why the reverse? Left side move opposite way from right so I would have to reverse it
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {
        double  fwd= -gamepad1.left_stick_y; // forward/back(Y axis)
        double str = gamepad1.left_stick_x;  // left/right(Strife/X axis)
        double rot = gamepad1.right_stick_x; // rotation from left stick(Spin go burrrrr)

        double tLPower = fwd + str + rot;
        double rLPower = fwd - str - rot;
        double tRPower = fwd - str + rot;
        double rRPower = fwd + str - rot;

        //You might not understand this but its important for the robot to move smoothly
        //Look at my notes for better understanding
        double max = Math.max(1.0, Math.abs(tLPower));
        max = Math.max(max, Math.abs(rLPower));
        max = Math.max(max, Math.abs(tRPower));
        max = Math.max(max, Math.abs(rRPower));

        //This gives all motor powers that "perfect ratio" so the robot moves smoothly to the input
        topLeftMotor.setPower(tLPower / max);
        topRightMotor.setPower(tRPower / max);
        rearLeftMotor.setPower(rLPower / max);
        rearRightMotor.setPower(rRPower / max);
    }
}
