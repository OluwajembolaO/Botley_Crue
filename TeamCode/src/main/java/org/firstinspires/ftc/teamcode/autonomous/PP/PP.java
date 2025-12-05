package org.firstinspires.ftc.teamcode.autonomous.PP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.Constants;

@Autonomous(name = "PP - Comp Auto", group = "Competition")
public class PP extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // Intake/Transfer/Outtake motors
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;

    // Shooting constants
    private static final double TICKS_PER_REV = 6000.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double TARGET_RPM = 4000;
    private static final double RPM_TOLERANCE = 200;
    private static final double SPINUP_TIME = 1.5; // seconds to spin up flywheels
    private static final double TRANSFER_TIME = 1.0; // seconds to run transfer motor
    private static final double INTAKE_POWER = 0.5;
    private static final double TRANSFER_POWER = 1.0;

    public enum PathState {
        //START POS_END POS
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE ARTIFACTS

        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,

        DRIVE_TO_POINT2,
        DRIVE_TO_BALL1,
        DRIVE_BALL1_TO_SHOOTPOS,
        SHOOT_BATCH1,

        DRIVE_TO_POINT5,
        DRIVE_TO_BALL2,
        DRIVE_BALL2_TO_SHOOTPOS,
        SHOOT_BATCH2,

        DRIVE_TO_POINT8,
        DRIVE_TO_BALL3,
        DRIVE_BALL3_TO_SHOOTPOS,
        SHOOT_BATCH3,

        DONE
    }

    public enum ShootingState {
        SPINUP,     // Spinning up flywheels to target RPM
        TRANSFER,   // Running transfer motor to shoot
        COMPLETE    // Shooting sequence complete
    }

    PathState pathState;
    ShootingState shootingState;
    private final Pose startPose = new Pose(23.10763209393346, 119.48336594911936, Math.toRadians(135));
    private final Pose shootPose = new Pose(50.16046966731898, 92.43052837573384, Math.toRadians(135));
    private final Pose point2 = new Pose(45.36986301369863, 84.54011741682974, Math.toRadians(180));
    private final Pose ballPose1 = new Pose(18.59882583170254, 84.54011741682974, Math.toRadians(180));
    private final Pose point5 = new Pose(42.833659491193735, 60.023483365949126, Math.toRadians(180));
    private final Pose ballPose2 = new Pose(19.471624266144815, 60.023483365949126, Math.toRadians(180));
    private final Pose point8 = new Pose(43.3972602739726, 35.22504892367907, Math.toRadians(180));
    private final Pose ballPose3 = new Pose(18.880626223091976, 35.22504892367907, Math.toRadians(180));

    private PathChain driveStartPosShootPos;
    private PathChain driveToPoint2, driveToBall1, driveBall1ToShootPos;
    private PathChain driveToPoint5, driveToBall2, driveBall2ToShootPos;
    private PathChain driveToPoint8, driveToBall3, driveBall3ToShootPos;
    public void buildPaths(){
        // Initial path: Start -> Shoot Position
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Batch 1: Shoot Pos -> Point 2 -> Ball 1 -> Shoot Pos
        driveToPoint2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, point2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), point2.getHeading())
                .build();

        driveToBall1 = follower.pathBuilder()
                .addPath(new BezierLine(point2, ballPose1))
                .setLinearHeadingInterpolation(point2.getHeading(), ballPose1.getHeading())
                .build();

        driveBall1ToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(ballPose1, shootPose))
                .setLinearHeadingInterpolation(ballPose1.getHeading(), shootPose.getHeading())
                .build();

        // Batch 2: Shoot Pos -> Point 5 -> Ball 2 -> Shoot Pos
        driveToPoint5 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, point5))
                .setLinearHeadingInterpolation(shootPose.getHeading(), point5.getHeading())
                .build();

        driveToBall2 = follower.pathBuilder()
                .addPath(new BezierLine(point5, ballPose2))
                .setLinearHeadingInterpolation(point5.getHeading(), ballPose2.getHeading())
                .build();

        driveBall2ToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(ballPose2, shootPose))
                .setLinearHeadingInterpolation(ballPose2.getHeading(), shootPose.getHeading())
                .build();

        // Batch 3: Shoot Pos -> Point 8 -> Ball 3 -> Shoot Pos
        driveToPoint8 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, point8))
                .setLinearHeadingInterpolation(shootPose.getHeading(), point8.getHeading())
                .build();

        driveToBall3 = follower.pathBuilder()
                .addPath(new BezierLine(point8, ballPose3))
                .setLinearHeadingInterpolation(point8.getHeading(), ballPose3.getHeading())
                .build();

        driveBall3ToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(ballPose3, shootPose))
                .setLinearHeadingInterpolation(ballPose3.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    runShootingSequence();
                    if (shootingState == ShootingState.COMPLETE) {
                        telemetry.addLine("Shooting Preload - Done");
                        follower.followPath(driveToPoint2, true);
                        setPathState(PathState.DRIVE_TO_POINT2);
                    }
                }
                break;

            // Ball Batch 1
            case DRIVE_TO_POINT2:
                if (!follower.isBusy()){
                    follower.followPath(driveToBall1, true);
                    setPathState(PathState.DRIVE_TO_BALL1);
                }
                break;

            case DRIVE_TO_BALL1:
                // Run intake while approaching and collecting ball
                intake.setPower(INTAKE_POWER);
                if (!follower.isBusy()){
                    telemetry.addLine("Collecting Ball 1");
                    follower.followPath(driveBall1ToShootPos, true);
                    setPathState(PathState.DRIVE_BALL1_TO_SHOOTPOS);
                }
                break;

            case DRIVE_BALL1_TO_SHOOTPOS:
                // Keep intake running to ensure ball is fully collected
                intake.setPower(INTAKE_POWER);
                if (!follower.isBusy()){
                    intake.setPower(0); // Stop intake before shooting
                    setPathState(PathState.SHOOT_BATCH1);
                }
                break;

            case SHOOT_BATCH1:
                if (!follower.isBusy()) {
                    runShootingSequence();
                    if (shootingState == ShootingState.COMPLETE) {
                        telemetry.addLine("Shooting Batch 1 - Done");
                        follower.followPath(driveToPoint5, true);
                        setPathState(PathState.DRIVE_TO_POINT5);
                    }
                }
                break;

            // Ball Batch 2
            case DRIVE_TO_POINT5:
                if (!follower.isBusy()){
                    follower.followPath(driveToBall2, true);
                    setPathState(PathState.DRIVE_TO_BALL2);
                }
                break;

            case DRIVE_TO_BALL2:
                // Run intake while approaching and collecting ball
                intake.setPower(INTAKE_POWER);
                if (!follower.isBusy()){
                    telemetry.addLine("Collecting Ball 2");
                    follower.followPath(driveBall2ToShootPos, true);
                    setPathState(PathState.DRIVE_BALL2_TO_SHOOTPOS);
                }
                break;

            case DRIVE_BALL2_TO_SHOOTPOS:
                // Keep intake running to ensure ball is fully collected
                intake.setPower(INTAKE_POWER);
                if (!follower.isBusy()){
                    intake.setPower(0); // Stop intake before shooting
                    setPathState(PathState.SHOOT_BATCH2);
                }
                break;

            case SHOOT_BATCH2:
                if (!follower.isBusy()) {
                    runShootingSequence();
                    if (shootingState == ShootingState.COMPLETE) {
                        telemetry.addLine("Shooting Batch 2 - Done");
                        follower.followPath(driveToPoint8, true);
                        setPathState(PathState.DRIVE_TO_POINT8);
                    }
                }
                break;

            // Ball Batch 3
            case DRIVE_TO_POINT8:
                if (!follower.isBusy()){
                    follower.followPath(driveToBall3, true);
                    setPathState(PathState.DRIVE_TO_BALL3);
                }
                break;

            case DRIVE_TO_BALL3:
                // Run intake while approaching and collecting ball
                intake.setPower(INTAKE_POWER);
                if (!follower.isBusy()){
                    telemetry.addLine("Collecting Ball 3");
                    follower.followPath(driveBall3ToShootPos, true);
                    setPathState(PathState.DRIVE_BALL3_TO_SHOOTPOS);
                }
                break;

            case DRIVE_BALL3_TO_SHOOTPOS:
                // Keep intake running to ensure ball is fully collected
                intake.setPower(INTAKE_POWER);
                if (!follower.isBusy()){
                    intake.setPower(0); // Stop intake before shooting
                    setPathState(PathState.SHOOT_BATCH3);
                }
                break;

            case SHOOT_BATCH3:
                if (!follower.isBusy()) {
                    runShootingSequence();
                    if (shootingState == ShootingState.COMPLETE) {
                        telemetry.addLine("Shooting Batch 3 - Done");
                        setPathState(PathState.DONE);
                    }
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

    public void runShootingSequence() {
        // Calculate target velocity in ticks per second
        double targetTicksPerSec = (TARGET_RPM * TICKS_PER_REV * GEAR_RATIO) / 60.0;

        switch (shootingState) {
            case SPINUP:
                // Spin up the flywheels
                outtake1.setVelocity(targetTicksPerSec);
                outtake2.setVelocity(targetTicksPerSec);
                transfer.setPower(0);
                intake.setPower(0);

                // Check if at target RPM or time has elapsed
                double rpm1 = (outtake1.getVelocity() / TICKS_PER_REV / GEAR_RATIO) * 60;
                double rpm2 = (outtake2.getVelocity() / TICKS_PER_REV / GEAR_RATIO) * 60;
                double avgRPM = (rpm1 + rpm2) / 2.0;

                if (pathTimer.getElapsedTimeSeconds() > SPINUP_TIME ||
                    Math.abs(avgRPM - TARGET_RPM) <= RPM_TOLERANCE) {
                    shootingState = ShootingState.TRANSFER;
                    pathTimer.resetTimer();
                    telemetry.addLine("Flywheels at speed - Shooting!");
                }
                telemetry.addData("Shooting State", "SPINUP");
                telemetry.addData("Target RPM", TARGET_RPM);
                telemetry.addData("Actual RPM", "%.0f", avgRPM);
                break;

            case TRANSFER:
                // Keep flywheels running and activate transfer
                outtake1.setVelocity(targetTicksPerSec);
                outtake2.setVelocity(targetTicksPerSec);
                transfer.setPower(TRANSFER_POWER);
                intake.setPower(0);

                if (pathTimer.getElapsedTimeSeconds() > TRANSFER_TIME) {
                    shootingState = ShootingState.COMPLETE;
                    pathTimer.resetTimer();
                }
                telemetry.addData("Shooting State", "TRANSFER");
                break;

            case COMPLETE:
                // Stop all motors
                outtake1.setVelocity(0);
                outtake2.setVelocity(0);
                transfer.setPower(0);
                intake.setPower(0);
                telemetry.addData("Shooting State", "COMPLETE");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        // Reset shooting state when entering a new shooting state
        if (newState == PathState.SHOOT_PRELOAD ||
            newState == PathState.SHOOT_BATCH1 ||
            newState == PathState.SHOOT_BATCH2 ||
            newState == PathState.SHOOT_BATCH3) {
            shootingState = ShootingState.SPINUP;
        }
    }
    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS; //First PathLine
        shootingState = ShootingState.SPINUP;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Initialize intake/transfer/outtake motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        // Set motor directions
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Enable velocity control on outtake motors
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Float for flywheels
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
