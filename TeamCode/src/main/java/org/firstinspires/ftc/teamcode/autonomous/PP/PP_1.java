package org.firstinspires.ftc.teamcode.autonomous.PP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="PP_1(TOP BLUE)")
public class PP_1 extends LinearOpMode {

    Functions robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Functions(hardwareMap);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        // ─────────── Move Forward (4 sec) ──────────          ─
        telemetry.addLine("State: MOVE_FORWARD (2 sec)");
        telemetry.update();
        robot.moveForward(0.2, 2.0);
        sleep(1000); // 1 second pause

        // ─────────── STOP ───────────
        telemetry.addLine("State: STOP (Autonomous Complete)");
        telemetry.update();
        robot.stop();
    }
}