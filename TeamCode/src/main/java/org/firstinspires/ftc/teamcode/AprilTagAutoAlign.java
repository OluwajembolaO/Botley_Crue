package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
    }


    @Override
    public void start() {
        limelight.start();
    }


    @Override
    public void loop() {
        // Get robot IMU yaw and update limelight
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());


        // Get Limelight result
        LLResult llResult = limelight.getLatestResult();


        // Default joystick inputs
        double fwd = -gamepad1.left_stick_y; // forward/backward
        double str = gamepad1.left_stick_x;  // strafe left/right
        double rot = gamepad1.right_stick_x; // rotation input from joystick


        boolean autoAlignActive = false; // flag if we override rotation


        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();


            double yaw = botPose.getOrientation().getYaw(AngleUnit.DEGREES);
            telemetry.addData("AprilTag botPose Yaw", yaw);


            double yawTolerance = 3.0; // degrees tolerance for alignment


            // Auto-align rotation control
            if (Math.abs(yaw) > yawTolerance) {
                autoAlignActive = true;
                // Simple proportional control for turning speed
                double kP = 0.01; // tune this gain as needed
                rot = -yaw * kP;


                // Clamp rot power to -0.3 to 0.3 to keep turning slow and controlled
                rot = Math.max(-0.3, Math.min(0.3, rot));


                telemetry.addData("Auto-align active", true);
                telemetry.addData("Rotation power override", rot);
            } else {
                telemetry.addData("Auto-align active", false);
                telemetry.addLine("Robot facing AprilTag!");
            }
        } else {
            telemetry.addLine("No valid AprilTag detected");
        }


        // Calculate mecanum motor powers combining joystick and/or auto rotation
        double tLPower = fwd + str + rot;
        double rLPower = fwd - str - rot;
        double tRPower = fwd - str + rot;
        double rRPower = fwd + str - rot;


        // Normalize motor powers
        double max = Math.max(1.0, Math.abs(tLPower));
        max = Math.max(max, Math.abs(rLPower));
        max = Math.max(max, Math.abs(tRPower));
        max = Math.max(max, Math.abs(rRPower));


        topLeftMotor.setPower(tLPower / max);
        topRightMotor.setPower(tRPower / max);
        rearLeftMotor.setPower(rLPower / max);
        rearRightMotor.setPower(rRPower / max);


        // Telemetry for debugging
        telemetry.addData("FWD", fwd);
        telemetry.addData("STR", str);
        telemetry.addData("ROT", rot);
        telemetry.update();
    }
}




