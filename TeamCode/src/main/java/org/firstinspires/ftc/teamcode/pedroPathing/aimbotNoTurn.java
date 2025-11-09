package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "aimbot >:D", group = "Autonomous")
@Config
public class aimbotNoTurn extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static double rotation = 0.0;
    public static double rotationTarget = 90.0;
    public static boolean cInterval = true;
    public final static boolean turnDeg = true;

    public static int initialX = 72;
    public static int initialY = 72;
    public static int initialR = 90;
    public final int initialRotationYaw = 0;
    public static double correctionInterval = 1000.0;
    private double currRobotYaw = 0.0;
    private double currRobotBearing = 0.0;
    private Pose3D cRP;




    // CAMERA STUFF
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            initialRotationYaw, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Timer correctionTimer;

//    private PathChain pathSev;

    private double ATPos() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                currRobotYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                currRobotBearing = detection.ftcPose.bearing;

                telemetry.addData("> ","yaw: %5.2f", currRobotYaw);
                telemetry.addData("> ","bearing: %5.2f", currRobotBearing);
                dashboardTelemetry.addData("> ","yaw: %5.2f", currRobotYaw);
                dashboardTelemetry.addData("> ","bearing: %5.2f", currRobotBearing);

                cRP = detection.robotPose;

                return detection.ftcPose.bearing;
            } else {
                // its an obelisk
                return -1.0;
            }
        }
        return -1.0;
    }


    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(1406.54, 1406.54, 658.234, 352.691)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1280, 720));
        builder.enableLiveView(false);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

    }



    // -----



    // PEDRO



    // -----
    private Follower follower;

    @Override
    public void init() {
        initAprilTag();
        telemetry.addData("> ","ready to follow");
        telemetry.update();
        follower = Constants.createFollower(hardwareMap);

        correctionTimer = new Timer();

//        final double initialHeading = ATPos();

        follower.setStartingPose(new Pose(initialX, initialY, Math.toRadians(initialR))); //set your starting pose
    }

    @Override
    public void start() {
        correctionTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        if (cInterval) {
            if (correctionTimer.getElapsedTime() < correctionInterval) {return;} else {correctionTimer.resetTimer();}
        }

// need start
        final double tagPosition = ATPos(); // does not update dashboard

//        if (tagPosition == -1.0) {
//            telemetry.addData("> ","no tags detected / obelisk");
//            telemetry.update();
//            return;
//        }
        telemetry.addData("> ","bearing: %5.2f", tagPosition);
        telemetry.addLine();
        dashboardTelemetry.addData("> ","bearing: %5.2f", tagPosition);
        dashboardTelemetry.addLine();
// need end




//        final double fialPoseYaw = cRP.getOrientation().getYaw(AngleUnit.DEGREES) + currRobotBearing;
        if (turnDeg) {
            dashboardTelemetry.addLine("turning by degrees");
            telemetry.addLine("turning by degrees");
            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(new Pose(initialX, initialY, Math.toRadians(follower.getPose().getHeading())), new Pose(initialX,initialY,Math.toRadians(rotation))))
                            .setConstantHeadingInterpolation(rotation)
                            .build()
            );
//            follower.turn(rotation, true);
        } else {
            dashboardTelemetry.addLine("turning to degrees");
            telemetry.addLine("turning to degrees");
            follower.turnTo(rotationTarget);
        }

        telemetry.addData("> ","---");
        dashboardTelemetry.addData("> ","---");
        telemetry.addData("> ","turning to %3.2f",rotation);
        dashboardTelemetry.addData("> ","turning to %3.2f",rotation);
        telemetry.addData("> ","elapsed time: %3.2f",(float) correctionTimer.getElapsedTime());
        dashboardTelemetry.addData("> ","elapsed time: %3.2f",(float) correctionTimer.getElapsedTime());
        telemetry.update();
        dashboardTelemetry.update();

    }

    @Override
    public void stop() {
        visionPortal.close();
        follower.breakFollowing();
    }
}