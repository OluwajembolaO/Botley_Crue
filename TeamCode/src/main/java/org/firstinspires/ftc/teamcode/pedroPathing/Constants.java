package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12) // Mass of robot (in Kilograms)
            //TODO
            .forwardZeroPowerAcceleration(-227.43694636130985)
            .lateralZeroPowerAcceleration(-84.23361789420737)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.065,0,0.0015,0.03)) //Its on PIDF on pp thing
            .headingPIDFCoefficients(new PIDFCoefficients(.715, 0, 0.0025, 0.03 ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.6,0,0.0001,0.6, 0.025)) //Also on thing
            .centripetalScaling(0.005)
    ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("motor2")
            .rightRearMotorName("motor4")
            .leftRearMotorName("motor3")
            .leftFrontMotorName("motor1")

            //TODO: Figure out which motors needs to be reversed 5:50 on the video - Jem

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //TODO A lot of testing - Jem
            .xVelocity(76.54722342904158)
            .yVelocity(65.90605463944084)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            //TODO: I need to tune this - Jem
            .forwardPodY(-5)
            .strafePodX(4.5)

            .distanceUnit(DistanceUnit.INCH) //Change units
            .hardwareMapName("pinpoint") //Change Name
            //TODO
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            //TODO Test encoders up +x left +y vice-versa -Jem
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1.4,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
