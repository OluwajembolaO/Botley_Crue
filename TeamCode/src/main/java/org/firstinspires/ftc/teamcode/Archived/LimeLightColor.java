package org.firstinspires.ftc.teamcode.Archived;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "LimeLightColor")
public class LimeLightColor extends OpMode {
    private Limelight3A limelight3A;
    private ElapsedTime timer;
    private int currentPipeline = 1;
    private static final double SWITCH_INTERVAL = 10.0; // seconds

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(1); // Start with pipeline 1 (green)
        limelight3A.start();
        timer = new ElapsedTime();
    }

    @Override
    public void start(){
        limelight3A.start();
        timer.reset(); // Reset timer when op mode starts
    }

    @Override
    public void loop() {
        // Check if 10 seconds have passed
        if (timer.seconds() >= SWITCH_INTERVAL) {
            // Switch pipeline
            currentPipeline = (currentPipeline == 1) ? 2 : 1;
            limelight3A.pipelineSwitch(currentPipeline);
            timer.reset(); // Reset timer for next switch
        }

        // Display current pipeline info
        if (currentPipeline == 1) {
            telemetry.addData("PIPELINE", "1: GREEN");
        } else {
            telemetry.addData("PIPELINE", "2: PURPLE");
        }

        telemetry.addData("Time until switch", "%.1f seconds", SWITCH_INTERVAL - timer.seconds());

        // Get and display limelight results
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()){
            telemetry.addData("Target X offset", llResult.getTx());
            telemetry.addData("Target Y offset", llResult.getTy());
            telemetry.addData("Target Area offset", llResult.getTa());
        } else {
            telemetry.addData("Status", "No valid target detected");
        }

        telemetry.update();
    }
}