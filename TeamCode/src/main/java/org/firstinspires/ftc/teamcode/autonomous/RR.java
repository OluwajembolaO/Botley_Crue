package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.util.List;

@Autonomous(name = "RR")
public class RR extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor turret;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize Turret Motor
        turret = hardwareMap.get(DcMotor.class, "turret"); // Change "turret" to your config name
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Or RUN_USING_ENCODER if you want speed control
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // or FLOAT
        // turret.setDirection(DcMotorSimple.Direction.REVERSE); // Uncomment if motor runs backwards

        // Start the Limelight polling for data
        limelight.pipelineSwitch(8); // LL
        limelight.start();

        // Initialize drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(56, -8, Math.toRadians(45 + 180)));

        int detectedTag = 0;

        // Scan for AprilTags during init
        while (!isStarted() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials =
                        result.getFiducialResults();

                if (!fiducials.isEmpty()) {
                    com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial =
                            fiducials.get(0);
                    int tagId = (int) fiducial.getFiducialId();

                    if (tagId == 21 || tagId == 22 || tagId == 23) {
                        detectedTag = tagId;
                        telemetry.addData("✓ Detected AprilTag", tagId);
                        telemetry.addData("✓ Selected Path", tagId - 20);
                        telemetry.addData("Status", "Ready to start!");
                    } else {
                        telemetry.addData("Unknown AprilTag", tagId);
                        telemetry.addData("Status", "Waiting for tags 21, 22, or 23...");
                    }
                } else {
                    telemetry.addData("AprilTag", "None detected");
                    telemetry.addData("Status", "Scanning...");
                }
            } else {
                telemetry.addData("Limelight", "No valid results");
                telemetry.addData("Status", "Scanning...");
            }

            telemetry.update();
            sleep(50);
        }

        waitForStart();

        if (isStopRequested()) return;

        // If no valid tag was detected, wait until one is found
        while (detectedTag == 0 && opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials =
                        result.getFiducialResults();

                if (!fiducials.isEmpty()) {
                    com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial =
                            fiducials.get(0);
                    int tagId = (int) fiducial.getFiducialId();

                    if (tagId == 21 || tagId == 22 || tagId == 23) {
                        detectedTag = tagId;
                        telemetry.addData("✓ AprilTag Found!", tagId);
                        telemetry.addData("Starting Path", tagId - 20);
                        telemetry.update();
                        sleep(500);
                        break;
                    }
                }
            }

            telemetry.addData("Status", "WAITING for AprilTag 21, 22, or 23");
            telemetry.addData("AprilTag", "None detected - Robot will not move");
            telemetry.update();
            sleep(100);
        }

        // If still no tag detected, exit
        if (detectedTag == 0) {
            telemetry.addData("Status", "No AprilTag detected - Stopping");
            telemetry.update();
            limelight.stop();
            return;
        }

        // ========== START TURRET SPINNING ==========
        turret.setPower(0.5); // Adjust power as needed (0.0 to 1.0)
        // ==========================================

        // Build and run the appropriate action based on detected tag
        Action trajectoryAction;

        switch (detectedTag) {
            case 21:
                // Path 1
                trajectoryAction = drive.actionBuilder(new Pose2d(56, -8, Math.toRadians(45 + 180)))
                        .splineTo(new Vector2d(35, -50), Math.toRadians(-100))
                        .build();
                break;

            case 22:
                // Path 2
                trajectoryAction = drive.actionBuilder(new Pose2d(56, -8, Math.toRadians(45 + 180)))
                        .splineTo(new Vector2d(12, 50), Math.PI / 2)
                        .build();
                break;

            case 23:
                // Path 3
                trajectoryAction = drive.actionBuilder(new Pose2d(56, -8, Math.toRadians(45 + 180)))
                        .splineTo(new Vector2d(-12, 50), Math.PI / 2)
                        .build();
                break;

            default:
                telemetry.addData("Error", "Invalid tag - No movement");
                telemetry.update();
                limelight.stop();
                turret.setPower(0); // Stop turret
                return;
        }

        // Run the selected trajectory (turret keeps spinning during this)
        Actions.runBlocking(trajectoryAction);

        // ========== STOP TURRET ==========
        turret.setPower(0);
        // =================================

        telemetry.addData("Status", "Complete");
        telemetry.update();

        // Stop Limelight
        limelight.stop();
    }
}