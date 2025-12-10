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

    // Path selection (1-4)
    private int selectedPath = 1;

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
        DRIVE_TO_SAMPLE1,      // For path 2
        DRIVE_TO_SAMPLE2,      // For path 2
        DRIVE_TO_WAYPOINT,     // For path 3
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

    // PATH 1 POSES (Original path)
    private final Pose startPose = new Pose(23.10763209393346, 119.48336594911936, Math.toRadians(135));
    private final Pose shootPose = new Pose(35.788649706457925, 107.36594911937378, Math.toRadians(135));
    private final Pose parkPose = new Pose(71.57729941291585, 132.44618395303328, Math.toRadians(270));

    // PATH 2 POSES (Sample collection path)
    private final Pose start2Pose = new Pose(23.10763209393346, 119.48336594911936, Math.toRadians(45));
    private final Pose sample1Pose = new Pose(50.0, 115.0, Math.toRadians(180));
    private final Pose sample2Pose = new Pose(60.0, 105.0, Math.toRadians(90));
    private final Pose shoot2Pose = new Pose(35.788649706457925, 107.36594911937378, Math.toRadians(135));
    private final Pose park2Pose = new Pose(60.0, 130.0, Math.toRadians(270));

    // PATH 3 POSES (Alternative shooting position)
    private final Pose start3Pose = new Pose(23.10763209393346, 119.48336594911936, Math.toRadians(135));
    private final Pose altShootPose = new Pose(45.0, 100.0, Math.toRadians(90));
    private final Pose waypoint3Pose = new Pose(55.0, 120.0, Math.toRadians(0));
    private final Pose park3Pose = new Pose(70.0, 125.0, Math.toRadians(180));

    // PATH 4 POSES (Fast park path)
    private final Pose start4Pose = new Pose(23.10763209393346, 119.48336594911936, Math.toRadians(135));
    private final Pose fastShootPose = new Pose(30.0, 110.0, Math.toRadians(135));
    private final Pose fastParkPose = new Pose(75.0, 135.0, Math.toRadians(270));

    private PathChain driveToShoot;
    private PathChain driveToPark;
    private PathChain additionalPath1;  // For paths with more waypoints
    private PathChain additionalPath2;  // For paths with more waypoints

    public void buildPaths() {
        switch (selectedPath) {
            case 1:
                // PATH 1: Original path - Start -> Shoot -> Park
                driveToShoot = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, shootPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                        .build();

                driveToPark = follower.pathBuilder()
                        .addPath(new BezierLine(shootPose, parkPose))
                        .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                        .build();
                break;

            case 2:
                // PATH 2: Sample collection path - Start -> Sample1 -> Sample2 -> Shoot -> Park
                driveToShoot = follower.pathBuilder()
                        .addPath(new BezierLine(start2Pose, sample1Pose))
                        .setLinearHeadingInterpolation(start2Pose.getHeading(), sample1Pose.getHeading())
                        .build();

                additionalPath1 = follower.pathBuilder()
                        .addPath(new BezierLine(sample1Pose, sample2Pose))
                        .setLinearHeadingInterpolation(sample1Pose.getHeading(), sample2Pose.getHeading())
                        .build();

                additionalPath2 = follower.pathBuilder()
                        .addPath(new BezierLine(sample2Pose, shoot2Pose))
                        .setLinearHeadingInterpolation(sample2Pose.getHeading(), shoot2Pose.getHeading())
                        .build();

                driveToPark = follower.pathBuilder()
                        .addPath(new BezierLine(shoot2Pose, park2Pose))
                        .setLinearHeadingInterpolation(shoot2Pose.getHeading(), park2Pose.getHeading())
                        .build();
                break;

            case 3:
                // PATH 3: Alternative shooting position - Start -> AltShoot -> Waypoint -> Park
                driveToShoot = follower.pathBuilder()
                        .addPath(new BezierLine(start3Pose, altShootPose))
                        .setLinearHeadingInterpolation(start3Pose.getHeading(), altShootPose.getHeading())
                        .build();

                additionalPath1 = follower.pathBuilder()
                        .addPath(new BezierLine(altShootPose, waypoint3Pose))
                        .setLinearHeadingInterpolation(altShootPose.getHeading(), waypoint3Pose.getHeading())
                        .build();

                driveToPark = follower.pathBuilder()
                        .addPath(new BezierLine(waypoint3Pose, park3Pose))
                        .setLinearHeadingInterpolation(waypoint3Pose.getHeading(), park3Pose.getHeading())
                        .build();
                break;

            case 4:
                // PATH 4: Fast park path - Start -> QuickShoot -> FastPark
                driveToShoot = follower.pathBuilder()
                        .addPath(new BezierLine(start4Pose, fastShootPose))
                        .setLinearHeadingInterpolation(start4Pose.getHeading(), fastShootPose.getHeading())
                        .build();

                driveToPark = follower.pathBuilder()
                        .addPath(new BezierLine(fastShootPose, fastParkPose))
                        .setLinearHeadingInterpolation(fastShootPose.getHeading(), fastParkPose.getHeading())
                        .build();
                break;

            default:
                // Default to path 1 if invalid selection
                selectedPath = 1;
                buildPaths();
                break;
        }
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_TO_SHOOT:
                // Wait for path to complete before transitioning
                if (!follower.isBusy()) {
                    // Different transitions based on selected path
                    if (selectedPath == 2) {
                        // Path 2: Go collect samples first
                        follower.followPath(additionalPath1, true);
                        setPathState(PathState.DRIVE_TO_SAMPLE1);
                    } else {
                        // Paths 1, 3, 4: Go to shooting
                        setPathState(PathState.SHOOT_BALLS);
                    }
                }
                break;

            case DRIVE_TO_SAMPLE1:
                // For Path 2: Drive to second sample
                if (!follower.isBusy()) {
                    follower.followPath(additionalPath2, true);
                    setPathState(PathState.DRIVE_TO_SAMPLE2);
                }
                break;

            case DRIVE_TO_SAMPLE2:
                // For Path 2: Drive to shoot position after collecting samples
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_BALLS);
                }
                break;

            case DRIVE_TO_WAYPOINT:
                // For Path 3: Go through waypoint before parking
                if (!follower.isBusy()) {
                    follower.followPath(driveToPark, true);
                    setPathState(PathState.DRIVE_TO_PARK);
                }
                break;

            case SHOOT_BALLS:
                // Run shooting sequence - robot stays stationary until complete
                runShootingSequence();
                if (shootingState == ShootingState.COMPLETE) {
                    telemetry.addLine("Shooting 3 Balls - Done");

                    // Different transitions based on selected path
                    if (selectedPath == 3) {
                        // Path 3: Go to waypoint first
                        follower.followPath(additionalPath1, true);
                        setPathState(PathState.DRIVE_TO_WAYPOINT);
                    } else {
                        // All other paths: Go directly to park
                        follower.followPath(driveToPark, true);
                        setPathState(PathState.DRIVE_TO_PARK);
                    }
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

        // Build paths and set start pose (will be updated in init_loop if path changes)
        buildPaths();
        setStartPose();

        // Initialize states
        pathState = PathState.DRIVE_TO_SHOOT;
        shootingState = ShootingState.IDLE;

        telemetry.addLine("Initialized - Use DPAD to select path (1-4)");
        telemetry.addData("Selected Path", selectedPath);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Allow path selection using gamepad
        if (gamepad1.dpad_up) {
            selectedPath = 1;
            buildPaths();
            setStartPose();
        } else if (gamepad1.dpad_right) {
            selectedPath = 2;
            buildPaths();
            setStartPose();
        } else if (gamepad1.dpad_down) {
            selectedPath = 3;
            buildPaths();
            setStartPose();
        } else if (gamepad1.dpad_left) {
            selectedPath = 4;
            buildPaths();
            setStartPose();
        }

        // Display selected path
        telemetry.addLine("=== PATH SELECTION ===");
        telemetry.addData("Selected Path", selectedPath);
        telemetry.addLine("");
        telemetry.addLine("DPAD UP    = Path 1 (Original)");
        telemetry.addLine("DPAD RIGHT = Path 2 (Sample Collection)");
        telemetry.addLine("DPAD DOWN  = Path 3 (Alt Shoot)");
        telemetry.addLine("DPAD LEFT  = Path 4 (Fast Park)");
        telemetry.update();
    }

    /**
     * Sets the starting pose based on selected path
     */
    private void setStartPose() {
        switch (selectedPath) {
            case 1:
                follower.setPose(startPose);
                break;
            case 2:
                follower.setPose(start2Pose);
                break;
            case 3:
                follower.setPose(start3Pose);
                break;
            case 4:
                follower.setPose(start4Pose);
                break;
        }
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
        telemetry.addData("Selected Path", selectedPath);
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