package org.firstinspires.ftc.teamcode.Libraries.pedroPathing;

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
            .mass(12.1) // Mass of robot (in Kilograms)
            //TODO
            .forwardZeroPowerAcceleration(-38.18046817305662)
            .lateralZeroPowerAcceleration(-66.45712898315684)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.066,0.000028,.01,.025)) //Its on PIDF on pp thing
            .headingPIDFCoefficients(new PIDFCoefficients(2,0.005,0.05,.000925))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0.00035,0.6,.015)) //Also on thing
            .centripetalScaling(0.0005)
            .translationalPIDFSwitch(3)
            .headingPIDFSwitch(.1570796327)
            .drivePIDFSwitch(15)
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
            .xVelocity(71.7694250392163)
            .yVelocity(55.82269913801058)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            //TODO: I need to tune this - Jem
            .forwardPodY(-1.5)
            .strafePodX(-5.5)

            .distanceUnit(DistanceUnit.INCH) //Change units
            .hardwareMapName("pinpoint") //Change Name

            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            //TODO Test encoders up +x left +y vice-versa -Jem
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1.2,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
