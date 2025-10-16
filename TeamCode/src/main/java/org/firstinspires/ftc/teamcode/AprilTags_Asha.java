/*package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class AprilTags_Asha extends OpMode {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    public int getTagID() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }

    public boolean hasValidTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1.0;
    }

    public boolean isColorPatternDetected(String pattern) {
        // Switch pipeline to the one corresponding to the color pattern
        int pipelineID = 0;
        switch (pattern) {
            case "purple green green":
                pipelineID = 1; // Color pattern pipeline 1
                break;
            case "green green purple":
                pipelineID = 2; // Color pattern pipeline 2
                break;
            default:
                return false;
        }

        // Switch pipeline
        limelightTable.getEntry("pipeline").setNumber(pipelineID);

        // Wait for pipeline switch to take effect (optional: add delay or check status)

        // Check if sufficient contours or blobs are detected
        double numTargets = limelightTable.getEntry("tcornx").getDoubleArray(new double[0]).length;

        return numTargets >= 3; // Expecting 3 blobs in correct sequence
    }

    public boolean shouldRecognizeTag() {
        if (!hasValidTarget()) return false;

        int tagID = getTagID();

        // For example, tag ID 5 must have "purple green green"
        if (tagID == 5) {
            return isColorPatternDetected("purple green green");
        }
        // Tag ID 9 must have "green green purple"
        else if (tagID == 9) {
            return isColorPatternDetected("green green purple");
        }

        return false;
    }


    }
}
*/

