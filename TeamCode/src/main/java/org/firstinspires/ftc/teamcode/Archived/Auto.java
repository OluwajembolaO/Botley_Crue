package org.firstinspires.ftc.teamcode.Archived;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto")
public class Auto extends LinearOpMode {

    Functions robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Functions(hardwareMap);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        // ─────────── Move Forward (4 sec) ───────────
        telemetry.addLine("State: MOVE_FORWARD (4 sec)");
        telemetry.update();
        robot.moveForward(0.75, 4.0);
        sleep(1000); // 1 second pause

        // ─────────── Move Backward (4 sec) ───────────
        telemetry.addLine("State: MOVE_BACKWARD (4 sec)");
        telemetry.update();
        robot.moveBackward(0.75, 4.0);
        sleep(1000);

        // ─────────── Move Forward (2 sec) ───────────
        telemetry.addLine("State: MOVE_FORWARD (2 sec)");
        telemetry.update();
        robot.moveForward(0.75, 2.0);
        sleep(1000);

        // ─────────── Slide Right (3 sec) ───────────
        telemetry.addLine("State: SLIDE_RIGHT (3 sec)");
        telemetry.update();
        robot.moveRight(0.75, 3.0);
        sleep(1000);

        // ─────────── Slide Left (3 sec) ───────────
        telemetry.addLine("State: SLIDE_LEFT (3 sec)");
        telemetry.update();
        robot.moveLeft(0.75, 3.0);
        sleep(1000);

        // ─────────── Turn Right (2 sec) ───────────
        telemetry.addLine("State: TURN_RIGHT (2 sec)");
        telemetry.update();
        robot.turnRight(0.75, 2.0);
        sleep(1000);

        // ─────────── Turn Left (3.5 sec) ───────────
        telemetry.addLine("State: TURN_LEFT (3.5 sec)");
        telemetry.update();
        robot.turnLeft(0.75, 3.5);
        sleep(1000);

        // ─────────── STOP ───────────
        telemetry.addLine("State: STOP (Autonomous Complete)");
        telemetry.update();
        robot.stop();
    }
}