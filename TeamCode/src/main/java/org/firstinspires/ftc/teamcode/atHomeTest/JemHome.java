package org.firstinspires.ftc.teamcode.atHomeTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="JemHome")
public class JemHome extends OpMode {

    // Sensors
    private Limelight3A limelight;
    private IMU imu;

    // Simple bang-bang control parameters
    private static final double DEADZONE = 3.0;  // degrees - if TX is within this, consider aligned

    @Override
    public void init() {
        // Sensor initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(RevOrientation));

        telemetry.addLine("AprilTag Alignment Guide Ready");
        telemetry.addLine("Will display alignment instructions");
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
        telemetry.addLine("=== APRILTAG ALIGNMENT ===");
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
                    // Tag is to the RIGHT - turn RIGHT
                    telemetry.addLine(">>> TURN RIGHT >>>>");
                    telemetry.addData("Action", "Rotate Clockwise");
                } else {
                    // Tag is to the LEFT - turn LEFT
                    telemetry.addLine("<<<< TURN LEFT <<<<");
                    telemetry.addData("Action", "Rotate Counter-Clockwise");
                }
                telemetry.addData("Offset", "%.2f° off center", Math.abs(tx));
            } else {
                // Tag is centered enough
                telemetry.addLine("✓✓✓ ALIGNED ✓✓✓");
                telemetry.addLine("✓✓✓  GOOD!  ✓✓✓");
                telemetry.addData("Status", "Centered within deadzone");
            }

        } else {
            telemetry.addLine("⚠ NO APRILTAG DETECTED ⚠");
            telemetry.addLine();
            telemetry.addLine("Searching for target...");
            telemetry.addData("Status", "Waiting for valid detection");
        }

        telemetry.update();
    }
}