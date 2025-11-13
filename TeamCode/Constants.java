package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static double AutonDelay = 0.5;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .forwardZeroPowerAcceleration(-644.1573425975088)
            .lateralZeroPowerAcceleration(-900.4536131149029);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("right_top")
            .rightRearMotorName("right_bottom")
            .leftRearMotorName("left_bottom")
            .leftFrontMotorName("left_top")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(1)
            .yVelocity(1);

    public static OTOSConstants OTlocalizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.DEGREES)
            .offset(new SparkFunOTOS.Pose2D(-7.5, 0.5, 180.0))
            .linearScalar(142.68123428571312)
            .angularScalar(0.01750454656217);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(DElocalizerConstants)
//                .OTOSLocalizer(OTlocalizerConstants)
                .build();
    }

    public static DriveEncoderConstants DElocalizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("right_top")
            .rightRearMotorName("right_bottom")
            .leftRearMotorName("left_bottom")
            .leftFrontMotorName("left_top")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.REVERSE)
            .robotWidth(9)
            .robotLength(14)
            .forwardTicksToInches(0.01780359287946125)
            .strafeTicksToInches(0.010001084649399318)
            .turnTicksToInches(0.020664314528815696);
}
