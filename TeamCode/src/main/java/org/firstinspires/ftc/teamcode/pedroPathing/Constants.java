package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathingNew.AsymMecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathingNew.AsymMecanumDriveConstants;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static double AutonDelay = 0.5;
    public static double blueDegrees = 122.0;
    public static double redDegrees  = 60.0;
    public static int AutonDistance = 72;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12);
//            .forwardZeroPowerAcceleration()
//            .lateralZeroPowerAcceleration();

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, 100, 1, 1
    );

    public static OTOSConstants otos = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-7.5, 0.5, Math.toRadians(90)))
            .linearScalar(2.325) //tune this
            .angularScalar(0.9961); //tune this

    public static AsymMecanumDriveConstants driveC = AsymMecanumDriveConstants.defaults();

    public static Follower createFollower(HardwareMap hw) {
        AsymMecanumDrive drive = new AsymMecanumDrive(hw, driveC);

        return new FollowerBuilder(followerConstants, hw)
                .setDrivetrain(drive)
                .OTOSLocalizer(otos)
                .pathConstraints(pathConstraints)
                .build();
    }
}
