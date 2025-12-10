package org.firstinspires.ftc.teamcode.autonomous.PP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.Constants;

@Autonomous(name = "PP - Path 3", group = "Competition")
public class PP_3 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVING,
        DONE
    }

    PathState pathState;

    // PATH 3 POSES - Simple straight line in X direction
    private final Pose startPose = new Pose(47.342465753424655, 12.399217221135029, Math.toRadians(90));
    private final Pose endPose = new Pose(14.371819960861057, 12.399217221135029, Math.toRadians(90));  // Move 90 units in X

    private PathChain drivePath;

    public void buildPaths() {
        // Simple straight line from start to end
        drivePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVING:
                // Wait for path to complete
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                telemetry.addLine("Autonomous Complete!");
                break;

            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

        // Initialize state
        pathState = PathState.DRIVING;

        telemetry.addLine("Path 3 - Ready to start");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();

        // Start the path
        follower.followPath(drivePath, true);
        pathState = PathState.DRIVING;

        telemetry.addLine("Starting Path 3...");
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        // Telemetry data
        Pose currentPose = follower.getPose();
        telemetry.addData("Path", "3");
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Path Time", "%.2f", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }
}
