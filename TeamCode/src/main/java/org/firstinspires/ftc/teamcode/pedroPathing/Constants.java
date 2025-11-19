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
            .mass(0) // Mass of robot (in Kilograms)
            //TODO
            //.forwardZeroPowerAcceleration(0)
            //.lateralZeroPowerAcceleration(0)
            //.translationalPIDFCoefficients(new PIDFCoefficients(0,0,0,0)) //Its on PIDF on pp thing
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0)) //Also on thing
            .centripetalScaling(0.000)
    ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("motor2")
            .rightRearMotorName("motor4")
            .leftRearMotorName("motor3")
            .leftFrontMotorName("motor1")

            //TODO: Figure out which motors needs to be reversed 5:50 on the video - Jem

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //TODO A lot of testing - Jem
            //.xVelocity(0)
            //.yVelocity(0)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            //TODO: I need to tune this - Jem
            .forwardPodY(-5)
            .strafePodX(0.5)

            .distanceUnit(DistanceUnit.INCH) //Change units
            .hardwareMapName("pinpoint") //Change Name
            //TODO
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            //TODO Test encoders up +x left +y vice-versa -Jem
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
