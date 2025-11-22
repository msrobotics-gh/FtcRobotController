package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathingNew.AsymDriveEncoderConstants;
import org.firstinspires.ftc.teamcode.pedroPathingNew.AsymDriveEncoderLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathingNew.AsymMecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathingNew.AsymMecanumDriveConstants;
import org.firstinspires.ftc.teamcode.pedroPathingNew.FusedLocalizer;

/**
 * Constants for multi-sensor fusion setup.
 * Combines OTOS, drive encoders, and AprilTags for robust localization.
 */
public class ConstantsFused {
    public static double AutonDelay = 0.5;
    public static double blueDegrees = 122.0;
    public static double redDegrees  = 60.0;
    public static int AutonDistance = 72;

    // Drive constants
    public static AsymMecanumDriveConstants driveC = AsymMecanumDriveConstants.defaults();

    // Encoder ticks per revolution for GoBILDA 5203 312 RPM motors
    private static final double TICKS_PER_REV = 537.7;
    // Wheel circumference in inches (for 96mm / ~3.78" wheels)
    private static final double WHEEL_CIRCUMFERENCE_INCHES = driveC.wheelRadiusMeters * 2 * Math.PI * 39.3701;
    // Ticks to inches conversion
    private static final double TICKS_TO_INCHES = WHEEL_CIRCUMFERENCE_INCHES / TICKS_PER_REV;

    // Asymmetric drive encoder constants (backup localizer)
    // Motor names and directions use defaults from AsymDriveEncoderConstants
    // which now match AsymMecanumDriveConstants defaults
    public static AsymDriveEncoderConstants encoderConstants = new AsymDriveEncoderConstants()
            // Ticks to inches conversions
            .forwardTicksToInches(TICKS_TO_INCHES)

            // Asymmetric strafe multipliers - TUNE THESE!
            // If robot strafes further left than right, decrease left multiplier
            // If robot strafes further right than left, decrease right multiplier
            .strafeTicksToInches(
                    TICKS_TO_INCHES * 1.0,  // Left strafe multiplier
                    TICKS_TO_INCHES * 1.0   // Right strafe multiplier
            )

            .turnTicksToInches(TICKS_TO_INCHES)

            // Robot geometry from AsymMecanumDriveConstants (convert meters to inches)
            .robotWidth(driveC.halfWidthFront * 2 * 39.3701)
            .robotLength(driveC.halfLengthX * 2 * 39.3701);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            // Heading PID tuning for asymmetric mecanum
            .headingPIDFCoefficients(new PIDFCoefficients(.5, 0, 0, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(.0, 0, 0.05, 0.01))
            .turnHeadingErrorThreshold(Math.PI / 20);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, 100, 1, 1
    );

    public static OTOSConstants otos = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-7.5, 0.5, Math.toRadians(90)))
            .linearScalar(2.325)
            .angularScalar(0.9961);

    /**
     * Create a follower with fused localization.
     * Combines OTOS (primary) with drive encoders (backup).
     */
    public static Follower createFusedFollower(HardwareMap hw) {
        AsymMecanumDrive drive = new AsymMecanumDrive(hw, driveC);

        // Create individual localizers
        OTOSLocalizer otosLoc = new OTOSLocalizer(hw, otos);
        AsymDriveEncoderLocalizer encoderLoc = new AsymDriveEncoderLocalizer(hw, encoderConstants);

        // Create fused localizer
        FusedLocalizer fusedLocalizer = new FusedLocalizer(otosLoc, encoderLoc);

        // Set fusion weights (70% OTOS, 30% encoders by default)
        fusedLocalizer.setFusionWeights(0.7);

        // TODO: Enable AprilTag correction if you have a camera
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