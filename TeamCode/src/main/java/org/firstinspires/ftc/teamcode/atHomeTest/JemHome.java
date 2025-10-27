package org.firstinspires.ftc.teamcode.atHomeTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="JemHome")
public class JemHome extends OpMode {

    // Sensors
    private Limelight3A limelight;
    private IMU imu;
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    // Simple bang-bang control parameters
    private static final double DEADZONE = 5.0;  // degrees - if TX is within this, consider aligned
    private static final double TURN_POWER = 0.15;  // Motor power for turning (adjust as needed)

    @Override
    public void init() {
        // Sensor initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        //Sets the motors
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        rearRightMotor = hardwareMap.get(DcMotor.class, "motor4");

        //Why the reverse? Left side move opposite way from right so I would have to reverse it
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(RevOrientation));

        telemetry.addLine("AprilTag Auto-Alignment Ready");
        telemetry.addLine("Robot will automatically align to AprilTag");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.pipelineSwitch(8);  // Set to pipeline 8 (AprilTag detection)
        limelight.start();
    }

    @Override
    public void loop() {
        // Get robot IMU yaw and update limelight
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        // Get Limelight result
        LLResult llResult = limelight.getLatestResult();

        // Clear previous telemetry
        telemetry.clear();
        telemetry.addLine("=== APRILTAG AUTO-ALIGNMENT ===");
        telemetry.addLine();

        // Auto-align logic - ALWAYS ACTIVE
        if (llResult != null && llResult.isValid()) {

            // Get TX - horizontal angle to target
            // Negative TX = tag is LEFT of center
            // Positive TX = tag is RIGHT of center
            double tx = llResult.getTx();

            telemetry.addData("Angle to Tag (TX)", "%.2f°", tx);
            telemetry.addData("Deadzone", "±%.1f°", DEADZONE);
            telemetry.addLine();

            // Simple decision: which way to turn?
            if (Math.abs(tx) > DEADZONE) {
                // Tag is NOT centered - need to turn
                if (tx > 0) {
                    // Tag is to the RIGHT - turn RIGHT (clockwise)
                    turnRight(TURN_POWER);
                    telemetry.addLine(">>> TURNING RIGHT >>>>");
                    telemetry.addData("Action", "Rotating Clockwise");
                } else {
                    // Tag is to the LEFT - turn LEFT (counter-clockwise)
                    turnLeft(TURN_POWER);
                    telemetry.addLine("<<<< TURNING LEFT <<<<");
                    telemetry.addData("Action", "Rotating Counter-Clockwise");
                }
                telemetry.addData("Offset", "%.2f° off center", Math.abs(tx));
            } else {
                // Tag is centered enough - STOP
                stopMotors();
                telemetry.addLine("✓✓✓ ALIGNED ✓✓✓");
                telemetry.addLine("✓✓✓  STOPPED  ✓✓✓");
                telemetry.addData("Status", "Centered within deadzone");
            }

        } else {
            // No AprilTag detected - STOP
            stopMotors();
            telemetry.addLine("⚠ NO APRILTAG DETECTED ⚠");
            telemetry.addLine();
            telemetry.addLine("Motors stopped - searching for target...");
            telemetry.addData("Status", "Waiting for valid detection");
        }

        telemetry.update();
    }

    /**
     * Turn the robot left (counter-clockwise)
     */
    private void turnLeft(double power) {
        topLeftMotor.setPower(power);
        topRightMotor.setPower(-power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(-power);
    }

    /**
     * Turn the robot right (clockwise)
     */
    private void turnRight(double power) {
        topLeftMotor.setPower(-power);
        topRightMotor.setPower(power);
        rearLeftMotor.setPower(-power);
        rearRightMotor.setPower(power);
    }

    /**
     * Stop all motors
     */
    private void stopMotors() {
        topLeftMotor.setPower(0);
        topRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }
}