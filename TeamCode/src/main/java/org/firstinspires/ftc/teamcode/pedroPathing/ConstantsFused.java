package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.Localizer;
import com.pedropathing.ftc.localization.constants.DriveEncoderLocalizerConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    // Drive encoder constants (backup localizer)
    public static DriveEncoderLocalizerConstants encoderConstants = new DriveEncoderLocalizerConstants()
            // Motor names (must match your hardware config)
            .setLeftFrontMotorName("front_left")
            .setLeftRearMotorName("back_left")
            .setRightFrontMotorName("front_right")
            .setRightRearMotorName("back_right")

            // Motor directions (should match AsymMecanumDriveConstants)
            .setLeftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .setLeftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .setRightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .setRightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            // Robot geometry (must match AsymMecanumDriveConstants)
            .setXMultiplier(1.0)    // Tune: forward encoder ticks to inches
            .setYMultiplier(1.0)    // Tune: lateral encoder ticks to inches

            // Track widths for mecanum kinematics
            .setForwardTicksToInches(1.0 / 537.7)  // GoBILDA 5203 312 RPM: 537.7 ticks/rev
            .setLateralTicksToInches(1.0 / 537.7)
            .setTrackWidth(AsymMecanumDriveConstants.defaults().halfWidthFront * 2)  // Use front width
            .setWheelRadius(AsymMecanumDriveConstants.defaults().wheelRadiusMeters * 39.3701); // Convert to inches

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            // Heading PID tuning for asymmetric mecanum
            .headingPIDF(0.5, 0, 0, 0.01)
            .secondaryHeadingPIDF(2.0, 0, 0.05, 0.01)
            .headingErrorThreshold(Math.PI / 20);

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
