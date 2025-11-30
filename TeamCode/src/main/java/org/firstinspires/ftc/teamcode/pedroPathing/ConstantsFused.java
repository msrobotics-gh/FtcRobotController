package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.Encoder;
//import com.pedropathing.ftc.localization.Localizer;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
//import com.pedropathing.ftc.localization.constants.DriveEncoderLocalizerConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.asymDrive.AsymMecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.asymDrive.AsymMecanumDriveConstants;

/**
 * Constants for multi-sensor fusion setup.
 * Combines OTOS, drive encoders, and AprilTags for robust localization.
 */
public class ConstantsFused {
    public static double AutonDelay = 0.5;
    public static double blueDegrees = 122.0;
    public static double redDegrees  = 60.0;
    public static int AutonDistance = 72;

    // Drive encoder constants (backup localizer)
    public static DriveEncoderConstants encoderConstants = new DriveEncoderConstants()
            // Motor names (must match your hardware config)
            .leftFrontMotorName("front_left")
            .leftRearMotorName("back_left")
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")

            // Motor directions (should match AsymMecanumDriveConstants)
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.REVERSE)
            .rightRearEncoderDirection(Encoder.REVERSE)

            // Robot geometry (must match AsymMecanumDriveConstants)
//            .xVelocity(1.0)    // Tune: forward encoder ticks to inches
//            .setYMultiplier(1.0)    // Tune: lateral encoder ticks to inches

            // Track widths for mecanum kinematics
            .forwardTicksToInches(1.0 / 537.7)  // GoBILDA 5203 312 RPM: 537.7 ticks/rev
            .strafeTicksToInches(1.0 / 537.7)
//            .setTrackWidth(AsymMecanumDriveConstants.defaults().halfWidthFront * 2)  // Use front width
//            .setWheelRadius(AsymMecanumDriveConstants.defaults().wheelRadiusMeters * 39.3701); // Convert to inches
            .robotLength(10.25)
            .robotWidth(6.5);



    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            // Heading PID tuning for asymmetric mecanum
            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.0, 0, 0.05, 0.01))
            .turnHeadingErrorThreshold(Math.PI / 20);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, 100, 1, 1
    );

    public static OTOSConstants otos = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-7.5, 0.5, Math.toRadians(90))) // AngleUnit.toRadians()?
            .linearScalar(2.325)
            .angularScalar(0.9961);

    public static AsymMecanumDriveConstants driveC = AsymMecanumDriveConstants.defaults();

    /**
     * Create a follower with fused localization.
     * Combines OTOS (primary) with drive encoders (backup).
     */
    public static Follower createFusedFollower(HardwareMap hw) {
        AsymMecanumDrive drive = new AsymMecanumDrive(hw, driveC);

        // Create individual localizers
        OTOSLocalizer otosLoc = new OTOSLocalizer(hw, otos);
        DriveEncoderLocalizer encoderLoc = new DriveEncoderLocalizer(hw, encoderConstants);

        // Create fused localizer
        FusedLocalizer fusedLocalizer = new FusedLocalizer(otosLoc, encoderLoc);

        // Set fusion weights (70% OTOS, 30% encoders by default)
        fusedLocalizer.setFusionWeights(0.7);

        // TODO: Enable AprilTag correction if you have a camera
        // im good
        // fusedLocalizer.enableAprilTagCorrection(aprilTagProcessor);

        return new FollowerBuilder(followerConstants, hw)
                .setDrivetrain(drive)
                .setLocalizer(fusedLocalizer)
                .pathConstraints(pathConstraints)
                .build();
    }

    /**
     * Create a follower with OTOS only (original setup).
     */
    public static Follower createFollower(HardwareMap hw) {
        AsymMecanumDrive drive = new AsymMecanumDrive(hw, driveC);

        return new FollowerBuilder(followerConstants, hw)
                .setDrivetrain(drive)
                .OTOSLocalizer(otos)
                .pathConstraints(pathConstraints)
                .build();
    }
}
