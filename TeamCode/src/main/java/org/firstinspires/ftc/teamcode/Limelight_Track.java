package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.autonomous.Functions;


import java.util.List;


@TeleOp(name = "limelight")
public class Limelight_Track extends LinearOpMode {


    private Limelight3A limelight;

    Functions robot;


    @Override
    public void runOpMode() throws InterruptedException
    {
        //This gets the limelight hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        robot = new Functions(hardwareMap);

        telemetry.setMsTransmissionInterval(11);


        limelight.pipelineSwitch(8); // 8 is april tags




        limelight.start();


        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());


            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {


                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));


                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());


                telemetry.addData("Botpose", botpose.toString());


                // Access barcode results
                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    telemetry.addData("Barcode", "Data: %s", br.getData());
                }




                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                for (LLResultTypes.ClassifierResult cr : classifierResults) {
                    telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                }



                //Recognize real world objects (AI)
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }



                //Gets pattern of April Tags
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());

                    // Check if AprilTag ID is 21
                    if (fr.getFiducialId() == 21) {
                        robot.moveForward(.7, 2.0);
                        telemetry.addData("DETECTED", "ID 21");
                    }

                    // Check if AprilTag ID is 22
                    if (fr.getFiducialId() == 22) {
                        robot.moveBackward(.7, 2.0);
                        telemetry.addData("DETECTED", "ID 22");
                    }

                    // Check if AprilTag ID is 23
                    if (fr.getFiducialId() == 23) {
                        robot.moveLeft(.7, 2.0);
                        telemetry.addData("DETECTED", "ID 23");
                    }
                    
                }
            }
            else {
                telemetry.addData("Limelight", "No data available");
            }

            //Checks for colors
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
            }

            telemetry.update();
        }

        limelight.stop();
    }
}