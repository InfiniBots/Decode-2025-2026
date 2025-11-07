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
            .mass(11.79)
            .forwardZeroPowerAcceleration(-44.75)
            .lateralZeroPowerAcceleration(-80)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.17,
                    0,
                    0.011,
                    0.01
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    2,
                    0,
                    0.1,
                    0
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    0,
                    0,
                    0,
                    0
            ))
            .headingPIDFSwitch(0.034906585)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(//Todo tune
                    0.005,
                    0,
                    0,
                    0.6,
                    0.06
            ));




    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(80)
            .yVelocity(60)
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4.495)
            .strafePodX(-3.38)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}

