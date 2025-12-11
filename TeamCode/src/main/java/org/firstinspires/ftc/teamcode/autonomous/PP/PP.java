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
    private Timer pathTimer, opModeTimer, actionTimer;

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
    private static final double SPINUP_TIME = 1.5;
    private static final double TRANSFER_TIME = 1.0;
    private static final double INTAKE_POWER = 0.8;
    private static final double TRANSFER_POWER = 1.0;
    private static final double INTAKE_COLLECT_TIME = 1.5;

    // All poses from compPath.pp (hardcoded)
    private final Pose startPose = new Pose(23.10763209393346, 119.48336594911936, Math.toRadians(135));
    private final Pose shootPoint1 = new Pose(50.16046966731898, 92.43052837573384, Math.toRadians(135));
    private final Pose point2 = new Pose(45.36986301369863, 84.54011741682974, Math.toRadians(180));
    private final Pose getBall1 = new Pose(18.59882583170254, 84.54011741682974, Math.toRadians(180));
    private final Pose shootPoint2 = new Pose(50.16046966731898, 92.43052837573384, Math.toRadians(135));
    private final Pose path5 = new Pose(42.833659491193735, 60.023483365949126, Math.toRadians(180));
    private final Pose getBall2 = new Pose(19.471624266144815, 60.023483365949126, Math.toRadians(180));
    private final Pose shootPoint3 = new Pose(50.16046966731898, 92.43052837573384, Math.toRadians(135));
    private final Pose path8 = new Pose(43.3972602739726, 35.22504892367907, Math.toRadians(180));
    private final Pose getBall3 = new Pose(18.880626223091976, 35.22504892367907, Math.toRadians(180));
    private final Pose shootPoint4 = new Pose(50.16046966731898, 92.43052837573384, Math.toRadians(135));

    // PathChains for each segment
    private PathChain toShootPoint1, toPoint2, toGetBall1, toShootPoint2, toPath5;
    private PathChain toGetBall2, toShootPoint3, toPath8, toGetBall3, toShootPoint4;

    public enum PathState {
        TO_SHOOT_1, SHOOTING_1,
        TO_POINT_2,
        TO_GET_BALL_1, COLLECTING_1,
        TO_SHOOT_2, SHOOTING_2,
        TO_PATH_5,
        TO_GET_BALL_2, COLLECTING_2,
        TO_SHOOT_3, SHOOTING_3,
        TO_PATH_8,
        TO_GET_BALL_3, COLLECTING_3,
        TO_SHOOT_4, SHOOTING_4,
        DONE
    }

    public enum ShootingState {
        IDLE, SPINUP, TRANSFER, COMPLETE
    }

    PathState pathState;
    ShootingState shootingState;

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        // Initialize motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Build all paths
        buildPaths();
        follower.setPose(startPose);

        pathState = PathState.TO_SHOOT_1;
        shootingState = ShootingState.IDLE;

        telemetry.addLine("PP Competition Auto Initialized");
        telemetry.addData("Total Segments", "11");
        telemetry.update();
    }

    private void buildPaths() {
        toShootPoint1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPoint1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPoint1.getHeading())
                .build();

        toPoint2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoint1, point2))
                .setLinearHeadingInterpolation(shootPoint1.getHeading(), point2.getHeading())
                .build();

        toGetBall1 = follower.pathBuilder()
                .addPath(new BezierLine(point2, getBall1))
                .setLinearHeadingInterpolation(point2.getHeading(), getBall1.getHeading())
                .build();

        toShootPoint2 = follower.pathBuilder()
                .addPath(new BezierLine(getBall1, shootPoint2))
                .setLinearHeadingInterpolation(getBall1.getHeading(), shootPoint2.getHeading())
                .build();

        toPath5 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoint2, path5))
                .setLinearHeadingInterpolation(shootPoint2.getHeading(), path5.getHeading())
                .build();

        toGetBall2 = follower.pathBuilder()
                .addPath(new BezierLine(path5, getBall2))
                .setLinearHeadingInterpolation(path5.getHeading(), getBall2.getHeading())
                .build();

        toShootPoint3 = follower.pathBuilder()
                .addPath(new BezierLine(getBall2, shootPoint3))
                .setLinearHeadingInterpolation(getBall2.getHeading(), shootPoint3.getHeading())
                .build();

        toPath8 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoint3, path8))
                .setLinearHeadingInterpolation(shootPoint3.getHeading(), path8.getHeading())
                .build();

        toGetBall3 = follower.pathBuilder()
                .addPath(new BezierLine(path8, getBall3))
                .setLinearHeadingInterpolation(path8.getHeading(), getBall3.getHeading())
                .build();

        toShootPoint4 = follower.pathBuilder()
                .addPath(new BezierLine(getBall3, shootPoint4))
                .setLinearHeadingInterpolation(getBall3.getHeading(), shootPoint4.getHeading())
                .build();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        follower.followPath(toShootPoint1, true);
    }

    @Override
    public void loop() {
        follower.update();
        stateUpdate();

        // Telemetry
        Pose currentPose = follower.getPose();
        telemetry.addData("State", pathState);
        telemetry.addData("Shooting", shootingState);
        telemetry.addData("Position", String.format("(%.1f, %.1f)", currentPose.getX(), currentPose.getY()));
        telemetry.addData("Heading", String.format("%.0f°", Math.toDegrees(currentPose.getHeading())));
        telemetry.addData("Time", String.format("%.1fs", opModeTimer.getElapsedTimeSeconds()));
        telemetry.update();
    }

    public void stateUpdate() {
        switch (pathState) {
            case TO_SHOOT_1:
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOTING_1;
                    shootingState = ShootingState.SPINUP;
                    actionTimer.resetTimer();
                }
                break;

            case SHOOTING_1:
                runShootingSequence();
                if (shootingState == ShootingState.COMPLETE) {
                    follower.followPath(toPoint2, true);
                    pathState = PathState.TO_POINT_2;
                }
                break;

            case TO_POINT_2:
                if (!follower.isBusy()) {
                    follower.followPath(toGetBall1, true);
                    pathState = PathState.TO_GET_BALL_1;
                }
                break;

            case TO_GET_BALL_1:
                if (!follower.isBusy()) {
                    pathState = PathState.COLLECTING_1;
                    intake.setPower(INTAKE_POWER);
                    actionTimer.resetTimer();
                }
                break;

            case COLLECTING_1:
                if (actionTimer.getElapsedTimeSeconds() >= INTAKE_COLLECT_TIME) {
                    intake.setPower(0);
                    follower.followPath(toShootPoint2, true);
                    pathState = PathState.TO_SHOOT_2;
                }
                break;

            case TO_SHOOT_2:
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOTING_2;
                    shootingState = ShootingState.SPINUP;
                    actionTimer.resetTimer();
                }
                break;

            case SHOOTING_2:
                runShootingSequence();
                if (shootingState == ShootingState.COMPLETE) {
                    follower.followPath(toPath5, true);
                    pathState = PathState.TO_PATH_5;
                }
                break;

            case TO_PATH_5:
                if (!follower.isBusy()) {
                    follower.followPath(toGetBall2, true);
                    pathState = PathState.TO_GET_BALL_2;
                }
                break;

            case TO_GET_BALL_2:
                if (!follower.isBusy()) {
                    pathState = PathState.COLLECTING_2;
                    intake.setPower(INTAKE_POWER);
                    actionTimer.resetTimer();
                }
                break;

            case COLLECTING_2:
                if (actionTimer.getElapsedTimeSeconds() >= INTAKE_COLLECT_TIME) {
                    intake.setPower(0);
                    follower.followPath(toShootPoint3, true);
                    pathState = PathState.TO_SHOOT_3;
                }
                break;

            case TO_SHOOT_3:
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOTING_3;
                    shootingState = ShootingState.SPINUP;
                    actionTimer.resetTimer();
                }
                break;

            case SHOOTING_3:
                runShootingSequence();
                if (shootingState == ShootingState.COMPLETE) {
                    follower.followPath(toPath8, true);
                    pathState = PathState.TO_PATH_8;
                }
                break;

            case TO_PATH_8:
                if (!follower.isBusy()) {
                    follower.followPath(toGetBall3, true);
                    pathState = PathState.TO_GET_BALL_3;
                }
                break;

            case TO_GET_BALL_3:
                if (!follower.isBusy()) {
                    pathState = PathState.COLLECTING_3;
                    intake.setPower(INTAKE_POWER);
                    actionTimer.resetTimer();
                }
                break;

            case COLLECTING_3:
                if (actionTimer.getElapsedTimeSeconds() >= INTAKE_COLLECT_TIME) {
                    intake.setPower(0);
                    follower.followPath(toShootPoint4, true);
                    pathState = PathState.TO_SHOOT_4;
                }
                break;

            case TO_SHOOT_4:
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOTING_4;
                    shootingState = ShootingState.SPINUP;
                    actionTimer.resetTimer();
                }
                break;

            case SHOOTING_4:
                runShootingSequence();
                if (shootingState == ShootingState.COMPLETE) {
                    pathState = PathState.DONE;
                }
                break;

            case DONE:
                stopAllMotors();
                telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
                break;
        }
    }

    public void runShootingSequence() {
        double targetTicksPerSec = (TARGET_RPM * TICKS_PER_REV * GEAR_RATIO) / 60.0;

        switch (shootingState) {
            case SPINUP:
                outtake1.setVelocity(targetTicksPerSec);
                outtake2.setVelocity(targetTicksPerSec);
                transfer.setPower(0);
                intake.setPower(0);

                double rpm1 = (outtake1.getVelocity() / TICKS_PER_REV / GEAR_RATIO) * 60;
                double rpm2 = (outtake2.getVelocity() / TICKS_PER_REV / GEAR_RATIO) * 60;
                double avgRPM = (rpm1 + rpm2) / 2.0;

                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME ||
                        Math.abs(avgRPM - TARGET_RPM) <= RPM_TOLERANCE) {
                    shootingState = ShootingState.TRANSFER;
                    actionTimer.resetTimer();
                }
                break;

            case TRANSFER:
                outtake1.setVelocity(targetTicksPerSec);
                outtake2.setVelocity(targetTicksPerSec);
                transfer.setPower(TRANSFER_POWER);

                if (actionTimer.getElapsedTimeSeconds() >= TRANSFER_TIME) {
                    shootingState = ShootingState.COMPLETE;
                }
                break;

            case COMPLETE:
                outtake1.setVelocity(0);
                outtake2.setVelocity(0);
                transfer.setPower(0);
                break;

            case IDLE:
                break;
        }
    }

    private void stopAllMotors() {
        intake.setPower(0);
        transfer.setPower(0);
        outtake1.setVelocity(0);
        outtake2.setVelocity(0);
    }
}
