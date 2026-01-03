package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.608)
            .forwardZeroPowerAcceleration(-33.539424856778865)
            .lateralZeroPowerAcceleration(-79.26255801701342)
            .translationalPIDFCoefficients(new PIDFCoefficients(.1,0,.01,.02))
            .translationalPIDFSwitch(4)
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,.02,.015))
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)


            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.008,0,.0001,.6,.003));

            //.centripetalScaling(.0005);


    public static PathConstraints pathConstraints = new PathConstraints( 0.99, 100, .7, 1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.75)
            .strafePodX(1)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            //leftFront: 0
            //rightFront: 1
            //leftRear: 2
            //rightRear: 3
            .maxPower(1)
            .xVelocity(85)

            .yVelocity(67.225062573049954)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
}