package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathingNew.AsymMecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathingNew.AsymMecanumDriveConstants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static double AutonDelay = 0.5;

    public static double blueDegrees = 33.0;
    public static double redDegrees  = -29.0;

    public static int AutonDistance = 36;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.0, 0, 0.05, 0.01))
            .turnHeadingErrorThreshold(Math.PI / 20)
            .forwardZeroPowerAcceleration(-644.1573425975088)
            .lateralZeroPowerAcceleration(-900.4536131149029);
//            .forwardZeroPowerAcceleration()
//            .lateralZeroPowerAcceleration();

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, 100, 1, 1
    );

    public static OTOSConstants otos = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-2.25, 0.5, Math.toRadians(0)))
            .linearScalar(1.416693); //tune this
//            .angularScalar(0.9961); //tune this

    public static AsymMecanumDriveConstants driveC = AsymMecanumDriveConstants.defaults();


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(115.91866902566092)
            .yVelocity(49.52288110523299);

    public static DriveEncoderConstants de = new DriveEncoderConstants()
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.REVERSE)
            .rightRearEncoderDirection(Encoder.REVERSE)
            .robotWidth(6.5)
            .robotLength(10.25)
            .forwardTicksToInches(0.01780359287946125)
            .strafeTicksToInches(0.010001084649399318)
            .turnTicksToInches(0.020664314528815696);

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.REVERSE)
            .rightRearEncoderDirection(Encoder.REVERSE)
            .robotWidth(6.5)
            .robotLength(10.25)
            .forwardTicksToInches(0.01780359287946125)
            .strafeTicksToInches(0.010001084649399318)
            .turnTicksToInches(0.020664314528815696);


    public static Follower createFollower2(HardwareMap hw) {
        AsymMecanumDrive drive = new AsymMecanumDrive(hw, driveC);

        return new FollowerBuilder(followerConstants, hw)
                .setDrivetrain(drive)
                .OTOSLocalizer(otos)
                .pathConstraints(pathConstraints)
                .build();
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                .build();
    }
}
