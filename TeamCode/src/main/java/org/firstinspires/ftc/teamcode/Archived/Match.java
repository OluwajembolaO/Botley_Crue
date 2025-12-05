package org.firstinspires.ftc.teamcode.Archived;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Match extends LinearOpMode {
    Functions2 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Functions2(hardwareMap);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        robot.startLL(); //Starts limelight


        waitForStart();
        //Step 1: Locate an april tag
        robot.turnToAprilTag(22);
        //Step 2: Align with april tag
        robot.alignWithAprilTag(22);
        //Step 3: Move to April tag;



    }
}
