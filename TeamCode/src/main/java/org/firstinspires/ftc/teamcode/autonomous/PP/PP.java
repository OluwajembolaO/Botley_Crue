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
    private Timer pathTimer, opModeTimer, shootTimer;

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
    private static final double SPINUP_TIME = 1.5;    // seconds to spin up flywheels
    private static final double TRANSFER_TIME = 1.0;  // seconds to run transfer motor
    private static final double INTAKE_POWER = 0.5;
    private static final double TRANSFER_POWER = 1.0;

    public enum PathState {
        DRIVE_TO_SHOOT,
        SHOOT_BALLS,
        DRIVE_TO_PARK,
        DONE
    }

    public enum ShootingState {
        IDLE,      // Not currently shooting
        SPINUP,    // Spinning up flywheels to target RPM
        TRANSFER,  // Running transfer motor to shoot
        COMPLETE   // Shooting sequence complete
    }

    PathState pathState;
    ShootingState shootingState;

    private final Pose startPose = new Pose(23.10763209393346, 119.48336594911936, Math.toRadians(135));
    private final Pose shootPose = new Pose(35.788649706457925, 107.36594911937378, Math.toRadians(135));
    private final Pose parkPose = new Pose(71.57729941291585, 132.44618395303328, Math.toRadians(270));

    private PathChain driveToShoot;
    private PathChain driveToPark;

    public void buildPaths() {
        // Path 1: Start -> Shoot Position
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Path 2: Shoot Position -> Park Position
        driveToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_TO_SHOOT:
                // Wait for path to complete before transitioning to shooting
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_BALLS);
                }
                break;

            case SHOOT_BALLS:
                // Run shooting sequence - robot stays stationary until complete
                runShootingSequence();
                if (shootingState == ShootingState.COMPLETE) {
                    telemetry.addLine("Shooting 3 Balls - Done");
                    follower.followPath(driveToPark, true);
                    setPathState(PathState.DRIVE_TO_PARK);
                }
                break;

            case DRIVE_TO_PARK:
                // Wait for path to complete
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                // Stop all motors to be safe
                stopAllMotors();
                telemetry.addLine("Autonomous Complete!");
                break;

            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }

    /**
     * Runs the shooting sequence state machine.
     * This method should be called repeatedly in the loop until shootingState == COMPLETE.
     * Uses shootTimer to track timing within the shooting sequence.
     */
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

                // Transition when either: time elapsed OR RPM is within tolerance
                if (shootTimer.getElapsedTimeSeconds() >= SPINUP_TIME ||
                        Math.abs(avgRPM - TARGET_RPM) <= RPM_TOLERANCE) {
                    shootingState = ShootingState.TRANSFER;
                    shootTimer.resetTimer(); // Reset timer for transfer phase
                    telemetry.addLine("Flywheels at speed - Shooting!");
                }

                telemetry.addData("Shooting State", "SPINUP");
                telemetry.addData("Target RPM", TARGET_RPM);
                telemetry.addData("Actual RPM", "%.0f", avgRPM);
                telemetry.addData("Spinup Time", "%.2f / %.2f", shootTimer.getElapsedTimeSeconds(), SPINUP_TIME);
                break;

            case TRANSFER:
                // Keep flywheels running and activate transfer
                outtake1.setVelocity(targetTicksPerSec);
                outtake2.setVelocity(targetTicksPerSec);
                transfer.setPower(TRANSFER_POWER);
                intake.setPower(0);

                // Transition when transfer time has elapsed
                if (shootTimer.getElapsedTimeSeconds() >= TRANSFER_TIME) {
                    shootingState = ShootingState.COMPLETE;
                    shootTimer.resetTimer();
                    telemetry.addLine("Transfer complete!");
                }

                telemetry.addData("Shooting State", "TRANSFER");
                telemetry.addData("Transfer Time", "%.2f / %.2f", shootTimer.getElapsedTimeSeconds(), TRANSFER_TIME);
                break;

            case COMPLETE:
                // Stop shooting motors (flywheels and transfer)
                outtake1.setVelocity(0);
                outtake2.setVelocity(0);
                transfer.setPower(0);
                // Don't stop intake here - let the path state control it

                telemetry.addData("Shooting State", "COMPLETE");
                break;

            case IDLE:
                // Do nothing - waiting for shooting to be triggered
                telemetry.addData("Shooting State", "IDLE");
                break;
        }
    }

    /**
     * Stops all motors safely
     */
    private void stopAllMotors() {
        intake.setPower(0);
        transfer.setPower(0);
        outtake1.setVelocity(0);
        outtake2.setVelocity(0);
    }

    /**
     * Sets the path state and resets appropriate timers.
     * Also initializes shooting state when entering a shooting phase.
     */
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

        // Reset shooting state when entering a shooting state
        if (newState == PathState.SHOOT_BALLS) {
            shootingState = ShootingState.SPINUP;
            shootTimer.resetTimer(); // Reset the shoot timer for the new sequence
        } else {
            // When not in a shooting state, set to IDLE
            shootingState = ShootingState.IDLE;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer(); // Separate timer for shooting sequence

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

        // Float for flywheels (coast when power is zero)
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set brake behavior for intake and transfer
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buildPaths();
        follower.setPose(startPose);

        // Initialize states
        pathState = PathState.DRIVE_TO_SHOOT;
        shootingState = ShootingState.IDLE;

        telemetry.addLine("Initialized - Ready to start");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        shootTimer.resetTimer();

        // Actually start the first path
        follower.followPath(driveToShoot, true);
        pathState = PathState.DRIVE_TO_SHOOT;

        telemetry.addLine("Starting autonomous...");
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        // Telemetry data
        Pose currentPose = follower.getPose();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Shooting State", shootingState);
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Path Time", "%.2f", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }
}