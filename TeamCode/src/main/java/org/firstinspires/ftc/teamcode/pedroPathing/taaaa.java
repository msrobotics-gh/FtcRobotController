package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * REFERENCE AUTONOMOUS - Pedro Pathing Best Practices
 *
 * This opmode demonstrates:
 * 1. AprilTag vision-based pose correction
 * 2. PathChain with multiple segments
 * 3. BezierCurve for smooth curved paths
 * 4. Different heading interpolation strategies
 * 5. Per-segment PathConstraints (fast travel vs. precise scoring)
 * 6. Semantic pose naming for clarity
 * 7. Comprehensive error handling and timeouts
 * 8. Dashboard visualization integration
 * 9. State machine for complex autonomous sequences
 * 10. Professional documentation and telemetry
 *
 * Scenario: Score specimen → Pick up samples → Score samples → Park
 *
 * @author FTC Team 9895
 * @version 1.0
 */
@Autonomous(name = "Reference Autonomous", group = "Pedro Examples")
@Config
public class taaaa extends OpMode {

    // ========== CONFIGURATION (Dashboard-editable) ==========
    public static double INITIAL_X = 72.0;
    public static double INITIAL_Y = 72.0;
    public static double INITIAL_HEADING_DEG = 90.0;
    public static boolean ENABLE_VISION_CORRECTION = true;
    public static double PATH_TIMEOUT_SECONDS = 15.0;
    public static double VISION_CORRECTION_INTERVAL = 2.0;  // Seconds between corrections

    // ========== PATH CONSTRAINTS ==========
    /**
     * Fast constraints for open-field navigation
     * Higher velocity and acceleration for efficiency
     */
    private static final PathConstraints FAST_CONSTRAINTS = new PathConstraints(
            1.0,    // maxVelocity (full speed)
            120.0,  // acceleration
            1.2,    // turnAcceleration
            1.0     // turnVelocity
    );

    /**
     * Precise constraints for scoring and delicate operations
     * Reduced velocity for accuracy
     */
    private static final PathConstraints PRECISE_CONSTRAINTS = new PathConstraints(
            0.6,    // maxVelocity (60% speed)
            60.0,   // acceleration
            0.8,    // turnAcceleration
            0.6     // turnVelocity
    );

    // ========== SEMANTIC POSE DEFINITIONS ==========
    // All measurements in inches, angles in degrees (converted to radians)

    /** Starting position - alliance-specific */
    private Pose startPose;

    /** Submersible scoring position for specimen */
    private final Pose specimenScorePose = new Pose(96, 96, Math.toRadians(90));

    /** First sample pickup location */
    private final Pose sample1PickupPose = new Pose(48, 48, Math.toRadians(180));

    /** Second sample pickup location */
    private final Pose sample2PickupPose = new Pose(48, 72, Math.toRadians(180));

    /** Observation zone scoring position */
    private final Pose observationScorePose = new Pose(72, 120, Math.toRadians(45));

    /** Final parking position */
    private final Pose parkPose = new Pose(60, 60, Math.toRadians(0));

    /** Control point for smooth curve from start to score */
    private final Pose scoreApproachControl = new Pose(84, 84);

    // ========== PATHS ==========
    private PathChain initialScorePath;
    private PathChain pickupPath;
    private PathChain scoreSamplesPath;
    private PathChain parkPath;

    // ========== HARDWARE ==========
    private Follower follower;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ========== STATE MANAGEMENT ==========
    private enum AutoState {
        VISION_INIT,           // Initialize and correct pose with AprilTag
        SCORE_SPECIMEN,        // Navigate to and score pre-loaded specimen
        PICKUP_SAMPLES,        // Navigate to and pickup samples
        SCORE_SAMPLES,         // Navigate to observation zone and score
        PARK,                  // Navigate to parking position
        HOLDING,               // Hold final position
        EMERGENCY_STOP         // Emergency state for errors
    }

    private AutoState currentState = AutoState.VISION_INIT;
    private ElapsedTime pathTimer;
    private ElapsedTime visionTimer;
    private ElapsedTime opmodeTimer;

    // ========== TELEMETRY ==========
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // ========== INITIALIZATION ==========

    @Override
    public void init() {
        // Initialize timers
        pathTimer = new ElapsedTime();
        visionTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();

        // Set starting pose from dashboard config
        startPose = new Pose(INITIAL_X, INITIAL_Y, Math.toRadians(INITIAL_HEADING_DEG));

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize AprilTag vision
        if (ENABLE_VISION_CORRECTION) {
            initAprilTag();
        }

        // Build all paths
        buildPaths();

        // Display initialization status
        telemetry.addLine("=== REFERENCE AUTONOMOUS ===");
        telemetry.addData("Status", "✓ Initialized");
        telemetry.addData("Starting Pose", "(%.1f, %.1f) @ %.0f°",
                INITIAL_X, INITIAL_Y, INITIAL_HEADING_DEG);
        telemetry.addData("Vision Correction", ENABLE_VISION_CORRECTION ? "ENABLED" : "DISABLED");
        telemetry.addLine();
        telemetry.addLine("Ready to start!");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        pathTimer.reset();
        visionTimer.reset();

        // Start with vision correction if enabled
        if (ENABLE_VISION_CORRECTION) {
            currentState = AutoState.VISION_INIT;
        } else {
            currentState = AutoState.SCORE_SPECIMEN;
        }
    }

    // ========== MAIN LOOP ==========

    @Override
    public void loop() {
        // CRITICAL: Update follower every loop
        follower.update();

        // Periodic vision correction during path following
        if (ENABLE_VISION_CORRECTION &&
            visionTimer.seconds() > VISION_CORRECTION_INTERVAL &&
            currentState != AutoState.VISION_INIT) {
            performVisionCorrection();
            visionTimer.reset();
        }

        // State machine
        autonomousStateMachine();

        // Update telemetry
        updateTelemetry();

        // Check for emergency timeout
        if (opmodeTimer.seconds() > 28.0) {  // Leave 2 seconds buffer before 30s limit
            telemetry.addData("⚠️ WARNING", "Approaching time limit!");
            if (opmodeTimer.seconds() > 29.0) {
                currentState = AutoState.EMERGENCY_STOP;
            }
        }
    }

    @Override
    public void stop() {
        // Clean shutdown
        if (visionPortal != null) {
            visionPortal.close();
        }
        follower.breakFollowing();
    }

    // ========== STATE MACHINE ==========

    /**
     * Main autonomous state machine
     * Handles transitions between different phases of autonomous
     */
    private void autonomousStateMachine() {
        switch (currentState) {
            case VISION_INIT:
                // Wait for first AprilTag detection to correct initial pose
                if (performVisionCorrection()) {
                    telemetry.addData("✓", "Vision correction complete");
                    transitionToState(AutoState.SCORE_SPECIMEN);
                } else if (pathTimer.seconds() > 2.0) {
                    // Timeout vision init after 2 seconds, proceed anyway
                    telemetry.addData("⚠️", "Vision init timeout, proceeding");
                    transitionToState(AutoState.SCORE_SPECIMEN);
                }
                break;

            case SCORE_SPECIMEN:
                // Start path to scoring position (only once)
                if (!follower.isBusy() && pathTimer.seconds() < 0.1) {
                    follower.followPath(initialScorePath, true);
                    pathTimer.reset();
                }

                // Check for completion or timeout
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    telemetry.addData("✓", "Specimen scored");
                    transitionToState(AutoState.PICKUP_SAMPLES);
                } else if (pathTimer.seconds() > PATH_TIMEOUT_SECONDS) {
                    handlePathTimeout("SCORE_SPECIMEN");
                }
                break;

            case PICKUP_SAMPLES:
                if (!follower.isBusy() && pathTimer.seconds() < 0.1) {
                    follower.followPath(pickupPath, false);
                    pathTimer.reset();
                }

                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    telemetry.addData("✓", "Samples collected");
                    transitionToState(AutoState.SCORE_SAMPLES);
                } else if (pathTimer.seconds() > PATH_TIMEOUT_SECONDS) {
                    handlePathTimeout("PICKUP_SAMPLES");
                }
                break;

            case SCORE_SAMPLES:
                if (!follower.isBusy() && pathTimer.seconds() < 0.1) {
                    follower.followPath(scoreSamplesPath, false);
                    pathTimer.reset();
                }

                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    telemetry.addData("✓", "Samples scored");
                    transitionToState(AutoState.PARK);
                } else if (pathTimer.seconds() > PATH_TIMEOUT_SECONDS) {
                    handlePathTimeout("SCORE_SAMPLES");
                }
                break;

            case PARK:
                if (!follower.isBusy() && pathTimer.seconds() < 0.1) {
                    follower.followPath(parkPath, true);
                    pathTimer.reset();
                }

                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    telemetry.addData("✓", "Parked successfully");
                    transitionToState(AutoState.HOLDING);
                } else if (pathTimer.seconds() > PATH_TIMEOUT_SECONDS) {
                    handlePathTimeout("PARK");
                }
                break;

            case HOLDING:
                // Hold final position
                follower.holdPoint(follower.getPose());
                telemetry.addData("Status", "✓ AUTONOMOUS COMPLETE");
                break;

            case EMERGENCY_STOP:
                // Emergency stop - halt all movement
                follower.breakFollowing();
                telemetry.addData("Status", "🚨 EMERGENCY STOP");
                telemetry.addData("Reason", "Timeout or critical error");
                break;
        }
    }

    /**
     * Handle state transitions with logging
     */
    private void transitionToState(AutoState newState) {
        currentState = newState;
        pathTimer.reset();
        telemetry.addData("→ Transition", newState.toString());
    }

    /**
     * Handle path timeout with emergency stop
     */
    private void handlePathTimeout(String pathName) {
        telemetry.addData("⚠️ ERROR", "Path timeout: " + pathName);
        telemetry.addData("Timeout", "%.1f seconds", PATH_TIMEOUT_SECONDS);
        follower.breakFollowing();
        currentState = AutoState.EMERGENCY_STOP;
    }

    // ========== PATH BUILDING ==========

    /**
     * Build all autonomous paths
     * Demonstrates different path types and constraints
     */
    private void buildPaths() {
        // PATH 1: Initial scoring path with smooth curve
        // Uses BezierCurve for smooth approach, precise constraints for accuracy
        initialScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startPose,
                        scoreApproachControl,  // Control point creates smooth S-curve
                        specimenScorePose
                ))
                .setConstraints(PRECISE_CONSTRAINTS)  // Slow and accurate
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        specimenScorePose.getHeading()
                )
                .build();

        // PATH 2: Pickup samples path
        // Fast travel with multiple waypoints, uses tangent heading
        pickupPath = follower.pathBuilder()
                // Fast travel to first sample
                .addPath(new BezierLine(specimenScorePose, sample1PickupPose))
                .setConstraints(FAST_CONSTRAINTS)
                .setTangentHeadingInterpolation()  // Face along path direction

                // Move to second sample (slower for precision)
                .addPath(new BezierLine(sample1PickupPose, sample2PickupPose))
                .setConstraints(PRECISE_CONSTRAINTS)
                .setConstantHeadingInterpolation(sample2PickupPose.getHeading())
                .build();

        // PATH 3: Score samples at observation zone
        // Smooth curve with constant heading
        scoreSamplesPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample2PickupPose,
                        new Pose(60, 96),  // Control point for smooth curve
                        observationScorePose
                ))
                .setConstraints(FAST_CONSTRAINTS)
                .setConstantHeadingInterpolation(Math.toRadians(45))  // Face corner
                .build();

        // PATH 4: Park in observation zone
        // Simple straight line with hold at end
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(observationScorePose, parkPose))
                .setConstraints(PRECISE_CONSTRAINTS)
                .setLinearHeadingInterpolation(
                        observationScorePose.getHeading(),
                        parkPose.getHeading()
                )
                .build();
    }

    // ========== APRILTAG VISION ==========

    /**
     * Initialize AprilTag processor with calibrated camera parameters
     * Camera calibration values from taa.java
     */
    private void initAprilTag() {
        // Camera position and orientation (robot-relative)
        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
                AngleUnit.DEGREES, 90, 0, 0, 0
        );

        // Create AprilTag processor with calibrated lens intrinsics
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(1406.54, 1406.54, 658.234, 352.691)  // fx, fy, cx, cy
                .build();

        // Build vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .enableLiveView(false)  // Disable preview for performance
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Perform vision-based pose correction using AprilTags
     * Returns true if correction was successful
     */
    private boolean performVisionCorrection() {
        if (aprilTag == null) return false;

        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            // Only use valid detections (not Obelisk tags)
            if (detection.metadata != null &&
                !detection.metadata.name.contains("Obelisk") &&
                detection.robotPose != null) {

                // Extract pose from AprilTag
                double x = detection.robotPose.getPosition().x;
                double y = detection.robotPose.getPosition().y;
                double yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                // Create corrected pose
                Pose correctedPose = new Pose(x, y, yaw);

                // Apply correction to follower
                follower.setPose(correctedPose);

                // Log correction
                telemetry.addData("Vision Correction", "Tag ID %d", detection.id);
                telemetry.addData("Corrected Pose", "(%.1f, %.1f) @ %.0f°",
                        x, y, Math.toDegrees(yaw));

                return true;
            }
        }

        return false;  // No valid detections
    }

    // ========== TELEMETRY ==========

    /**
     * Comprehensive telemetry update
     * Displays state, position, timing, and diagnostics
     */
    private void updateTelemetry() {
        Pose currentPose = follower.getPose();

        // === DRIVER STATION TELEMETRY ===
        telemetry.addLine("=== REFERENCE AUTONOMOUS ===");
        telemetry.addLine();

        // State information
        telemetry.addData("State", currentState.toString());
        telemetry.addData("Path Active", follower.isBusy() ? "YES" : "NO");
        telemetry.addLine();

        // Robot position
        telemetry.addData("Position", "(%.1f, %.1f)",
                currentPose.getX(), currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addLine();

        // Timing
        telemetry.addData("Opmode Time", "%.1fs", opmodeTimer.seconds());
        telemetry.addData("Path Time", "%.1fs", pathTimer.seconds());

        // Vision status
        if (ENABLE_VISION_CORRECTION && aprilTag != null) {
            int tagCount = aprilTag.getDetections().size();
            telemetry.addData("AprilTags", "%d detected", tagCount);
        }

        telemetry.update();

        // === DASHBOARD TELEMETRY ===
        dashboardTelemetry.addData("state", currentState.toString());
        dashboardTelemetry.addData("x", currentPose.getX());
        dashboardTelemetry.addData("y", currentPose.getY());
        dashboardTelemetry.addData("heading_deg", Math.toDegrees(currentPose.getHeading()));
        dashboardTelemetry.addData("is_busy", follower.isBusy());
        dashboardTelemetry.addData("time", opmodeTimer.seconds());
        dashboardTelemetry.update();
    }
}
