package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "apriltag 1", group = "Autonomous")
//@Config
public class taa extends OpMode {
    public final int initialRotationYaw = 0;
    // CAMERA STUFF
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            initialRotationYaw, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private double ATPos() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
//                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                            detection.robotPose.getPosition().x,
//                            detection.robotPose.getPosition().y,
//                            detection.robotPose.getPosition().z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    return detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                }
            } else {
                // its an obelisk
                return 0.0;
            }
        }
        return -1;
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

    @Override
    public void init() {
//        camera = hardwareMap.get(Limelight3A.class, "limelight");
        initAprilTag();
        telemetry.addData("> ","ready to follow");
        telemetry.update();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose()); //set your starting pose
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        follower.update();
//        if (dontFryCpu < 10000) {dontFryCpu++; return;} else {dontFryCpu = 0;}
        // running this loop hundreds of time per ms is kinda excessive ._.

        //if you're not using limelight you can follow the same steps: build an offset pose, put your heading offset, and generate a path etc

        final double tagPosition = ATPos();

        if (tagPosition == -1) {
            telemetry.addData("> ","no tags detected");
            telemetry.update();
            return;
        }

//        if (!following) {
//            follower.followPath(
//                    follower.pathBuilder()
////                            .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
//                            .addPath(new BezierLine(follower.getPose(), follower.getPose()))
//                            .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(tagPosition))
//                            .build()
//            );
//            telemetry.addData("> ","following "+tagPosition);
//            telemetry.update();
//        }

        final Pose currentPose = follower.getPose();
        final Pose tagPose = new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(tagPosition));

        //This uses the aprilTag to relocalize your robot
        //You can also create a custom AprilTag fusion Localizer for the follower if you want to use this by default for all your autos
        follower.setPose(tagPose);
        telemetry.addData("> ","following %f",tagPosition);
        telemetry.update();

//        if (following && !follower.isBusy()) following = false;
    }

    @Override
    public void stop() {
        visionPortal.close();
    }

//    private Pose getRobotPoseFromCamera() {
//        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
//        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
//
//        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
//        return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
//    }
}