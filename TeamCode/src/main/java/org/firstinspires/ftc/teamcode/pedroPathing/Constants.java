package org.firstinspires.ftc.teamcode.pedroPathing;

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

// Please update constants when robot goes through a major change and especially when motors change.

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.4) // NOT ACTUAL MASS JUST EXAMPLE, MEASURE ROBOT IN KILOGRAMS
            .forwardZeroPowerAcceleration(-26.9019699405074)     // ADD: Number from automatic tuner
            .lateralZeroPowerAcceleration(-72.20540123644508)   // ADD: Number from automatic tuner
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0.001, 0.035))
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")      // Match YOUR motor names
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD) // Remove once forward & lateral velocity are added
            .xVelocity(71.00797301765502)    // ADD: Forward velocity from tuner
            .yVelocity(47.84892308993602);   // ADD: Lateral velocity from tuner

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-8.125) // NOT ACTUAL MEASUREMENT, MEASURE IT IN INCHES
            .strafePodX(-6) // NOT ACTUAL MEASUREMENT, MEASURE IT IN INCHES
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("Pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}