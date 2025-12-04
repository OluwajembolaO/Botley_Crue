package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class PP extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //START POS_END POS
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE ARTIFACTS

        DRIVE_STARTPOS_SHOOT_POS,

        SHOOT_PRELOAD,
        DRIVESHOOTPOS_END_POS,

    }

    PathState pathState;
    private final Pose startPose = new Pose(23.10763209393346, 119.48336594911936, Math.toRadians(135));
    private final Pose shootPose = new Pose(50.16046966731898,92.43052837573384,Math.toRadians(135));

    private final Pose endPose = new Pose(45.36986301369863, 84.54011741682974, Math.toRadians(180));
    private PathChain driveStartPosShootPos, driveShootPosEndPos;
    public void buildPaths(){
        //put coords of starting pos and ending pos
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                //Has follower done its path
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    //Shoot balls
                    telemetry.addLine("Shooting");
                    follower.followPath(driveShootPosEndPos);
                    setPathState(PathState.DRIVESHOOTPOS_END_POS);

                }
                break;
            case DRIVESHOOTPOS_END_POS:
                if (!follower.isBusy()){
                    telemetry.addLine("Done with commands");
                }
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
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add other init like limelight

        buildPaths();
        follower.setPose(startPose);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }
    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        // Telemetry data
        Pose currentPose = follower.getPose();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
