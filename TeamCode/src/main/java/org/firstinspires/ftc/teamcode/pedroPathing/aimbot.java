package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.paths.PathChain;

import android.util.Size;

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
public class aimbot extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static double rotation = 0.0;
    public static boolean cInterval = true;

    private final int initialX = 72;
    private final int initialY = 72;
    private final int initialR = 90;
    public final int initialRotationYaw = 0;
    private final double correctionInterval = 1000.0;
    // CAMERA STUFF
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            initialRotationYaw, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Timer correctionTimer;
    private double currRobotYaw = 0.0;
    private double currRobotBearing = 0.0;
    private Pose3D cRP;

//    private PathChain pathSev;

    private double ATPos() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                currRobotYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                currRobotBearing = detection.ftcPose.bearing;

                dashboardTelemetry.addData("> ","yaw: %5.2f", currRobotYaw);
                dashboardTelemetry.addData("> ","bearing: %5.2f", currRobotBearing);
//                dashboardTelemetry.update();

                cRP = detection.robotPose;
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                //  && detection.robotPose != null
//                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                            detection.robotPose.getPosition().x,
//                            detection.robotPose.getPosition().y,
//                            detection.robotPose.getPosition().z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//                    return detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                    return detection.ftcPose.bearing;
            } else {
                // its an obelisk
                return -1.0;
            }
        }
        return -1.0;
    }   // end method telemetryAprilTag()


    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                // == CAMERA CALIBRATION ==
                .setLensIntrinsics(1406.54, 1406.54, 658.234, 352.691)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(false); // disables camera view

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


















    // PEDRO STUFF
    private Follower follower;
//    private boolean following = false;
//    private final Pose TARGET_LOCATION = new Pose(); //Put the target location here

    int dontFryCpu = 0;

    private Pose initPose;

    @Override
    public void init() {
//        camera = hardwareMap.get(Limelight3A.class, "limelight");
        initAprilTag();
        telemetry.addData("> ","ready to follow");
        telemetry.update();
        follower = Constants.createFollower(hardwareMap);

        correctionTimer = new Timer();

        final double initialHeading = ATPos();

        follower.setStartingPose(new Pose(initialX, initialY, Math.toRadians(initialR))); //set your starting pose
    }

    @Override
    public void start() {
        correctionTimer.resetTimer();
    }

    int kill = 0;

    @Override
    public void loop() {
        follower.update();
        if (cInterval) {
            if (correctionTimer.getElapsedTime() < correctionInterval) {return;} else {correctionTimer.resetTimer();}
        }
        // running this loop hundreds of time per ms is kinda excessive ._.

// need start
//        final double tagPosition = ATPos(); // does not update dashboard
//
//        if (tagPosition == -1.0) {
//            telemetry.addData("> ","no tags detected / obelisk");
//            telemetry.update();
//            return;
//        }
// need end




//        final double fialPoseYaw = cRP.getOrientation().getYaw(AngleUnit.DEGREES) + currRobotBearing;
        follower.turn(rotation, true);

        dashboardTelemetry.addData("> ","---");
        telemetry.addData("> ","turning to %3.2f",rotation);
        dashboardTelemetry.addData("> ","turning to %3.2f",rotation);
        telemetry.addData("> ","elapsed time: %3.2f",correctionTimer.getElapsedTime());
        dashboardTelemetry.addData("> ","elapsed time: %3.2f",correctionTimer.getElapsedTime());
        telemetry.update();
        dashboardTelemetry.update();

//        if (following && !follower.isBusy()) following = false;
    }

    @Override
    public void stop() {
        visionPortal.close();
        follower.breakFollowing();
    }

//    private Pose getRobotPoseFromCamera() {
//        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
//        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
//
//        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
//        return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
//    }
}