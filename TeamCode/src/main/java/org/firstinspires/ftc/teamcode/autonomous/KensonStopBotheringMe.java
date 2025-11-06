package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

@Autonomous(name="KensonStopBotheringMe")
public class KensonStopBotheringMe extends LinearOpMode {

    Functions robot;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Functions(hardwareMap);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // 8 is april tags
        limelight.start();

        telemetry.addLine("Initialized. Scanning for AprilTags...");
        telemetry.update();

        // Scan for AprilTag before start
        int detectedTagId = -1;
        while (!isStarted() && !isStopRequested()) {
            detectedTagId = detectAprilTag();
            if (detectedTagId != -1) {
                telemetry.addData("Detected AprilTag", detectedTagId);
            } else {
                telemetry.addLine("No AprilTag detected (21, 22, or 23)");
            }
            telemetry.addLine("Ready to start!");
            telemetry.update();
            sleep(100);
        }

        waitForStart();

        // If no tag was detected during init, try one more time
        if (detectedTagId == -1) {
            telemetry.addLine("Scanning one more time...");
            telemetry.update();
            sleep(500); // Give camera time to stabilize
            detectedTagId = detectAprilTag();
        }

        // Execute sequence based on detected AprilTag - will run to completion
        if (detectedTagId == 21) {
            executeSequence21();
            telemetry.addLine("Sequence 21 Complete!");
        } else if (detectedTagId == 22) {
            executeSequence22();
            telemetry.addLine("Sequence 22 Complete!");
        } else if (detectedTagId == 23) {
            executeSequence23();
            telemetry.addLine("Sequence 23 Complete!");
        } else {
            telemetry.addLine("ERROR: No valid AprilTag detected!");
            telemetry.addLine("No sequence executed.");
        }

        telemetry.update();

        robot.stop();
        limelight.stop();
    }

    private int detectAprilTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int tagId = (int) fr.getFiducialId();
                // Only return tags 21, 22, or 23
                if (tagId == 21 || tagId == 22 || tagId == 23) {
                    return tagId;
                }
            }
        }
        return -1; // No valid tag detected
    }

    // Sequence for AprilTag 21 - Runs to completion without interruption
    private void executeSequence21() {
        telemetry.addLine("=== EXECUTING SEQUENCE 21 ===");
        telemetry.update();

        robot.moveForward(0.75, 4.0);
        sleep(500);

        robot.turnLeft(0.75, 2.0);
        sleep(500);

        robot.moveForward(0.75, 2.0);
        sleep(500);

        robot.moveRight(0.75, 1.5);
        sleep(500);
    }

    // Sequence for AprilTag 22 - Runs to completion without interruption
    private void executeSequence22() {
        telemetry.addLine("=== EXECUTING SEQUENCE 22 ===");
        telemetry.update();

        robot.moveForward(0.75, 4.0);
        sleep(500);

        robot.moveRight(0.75, 3.0);
        sleep(500);

        robot.moveBackward(0.75, 2.0);
        sleep(500);

        robot.turnRight(0.75, 1.5);
        sleep(500);
    }

    // Sequence for AprilTag 23 - Runs to completion without interruption
    private void executeSequence23() {
        telemetry.addLine("=== EXECUTING SEQUENCE 23 ===");
        telemetry.update();

        robot.moveForward(0.75, 4.0);
        sleep(500);

        robot.turnRight(0.75, 2.0);
        sleep(500);

        robot.moveForward(0.75, 2.0);
        sleep(500);

        robot.moveLeft(0.75, 1.5);
        sleep(500);
    }


}