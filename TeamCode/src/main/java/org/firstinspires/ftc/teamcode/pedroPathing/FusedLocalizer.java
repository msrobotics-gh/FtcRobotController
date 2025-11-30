package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Multi-sensor fusion localizer combining OTOS, drive encoders, and AprilTags.
 *
 * Architecture:
 * - Primary: OTOS for continuous tracking
 * - Backup: Drive encoders when OTOS fails/unreliable
 * - Corrections: AprilTags for absolute position resets
 *
 * Fusion Strategy:
 * 1. Complementary Filter: Blend OTOS and encoder readings
 * 2. Outlier Detection: Detect and reject bad OTOS readings
 * 3. AprilTag Corrections: Reset position when tags detected with high confidence
 */
public class FusedLocalizer implements Localizer {

    // Primary localizers
    private final Localizer otosLocalizer;
    private final Localizer encoderLocalizer;

    // AprilTag processor (optional)
    private AprilTagProcessor aprilTagProcessor;

    // Fusion parameters
    private double otosWeight = 0.7;           // Trust OTOS 70% by default

    private double encoderWeight = 1.0 - otosWeight;        // Trust encoders 30% by default
    private boolean useAprilTagCorrection = false;

    // Outlier detection
    private static final double MAX_VELOCITY_JUMP = 100.0;  // inches/sec
    private static final double MAX_POSITION_JUMP = 10.0;   // inches
    private Pose previousPose = new Pose();
//    private Vector previousVelocity = new Vector();
    private long previousTimeNanos = System.nanoTime();

    // AprilTag correction
    private static final double APRILTAG_CONFIDENCE_THRESHOLD = 0.8;
    private static final double APRILTAG_CORRECTION_WEIGHT = 0.5;
    private boolean aprilTagCorrectionEnabled = true;

    // Reliability tracking
    private double otosReliability = 1.0;  // 0.0 to 1.0
    private static final double RELIABILITY_DECAY = 0.99;
    private static final double RELIABILITY_RECOVERY = 0.05;

    // Current pose tracking
    private Pose currentPose = new Pose();
    private Pose startPose = new Pose();
    private double totalHeading = 0.0;

    /**
     * Creates a fused localizer.
     *
     * @param otosLocalizer Primary OTOS localizer
     * @param encoderLocalizer Backup encoder localizer (can be null)
     */
    public FusedLocalizer(Localizer otosLocalizer, Localizer encoderLocalizer) {
        this.otosLocalizer = otosLocalizer;
        this.encoderLocalizer = encoderLocalizer;
    }

    /**
     * Enable AprilTag corrections.
     *
     * @param processor AprilTag processor from vision portal
     */
    public void enableAprilTagCorrection(AprilTagProcessor processor) {
        this.aprilTagProcessor = processor;
        this.useAprilTagCorrection = true;
    }

    /**
     * Set the fusion weights.
     *
     * @param otosWeight Weight for OTOS (0.0 to 1.0)
     */
    public void setFusionWeights(double otosWeight) {
        this.otosWeight = Math.max(0.0, Math.min(1.0, otosWeight));
        this.encoderWeight = 1.0 - this.otosWeight; // WOAH ITS HERE AGAIN
    }

    @Override
    public Pose getPose() {
        // Get readings from all sensors
        Pose otosPose = otosLocalizer.getPose();
        Pose encoderPose = encoderLocalizer != null ? encoderLocalizer.getPose() : otosPose;

        // Detect OTOS outliers
        detectOutliers(otosPose);

        // Blend OTOS and encoders based on reliability
        double adaptiveOtosWeight = otosWeight * otosReliability;
        double adaptiveEncoderWeight = 1.0 - adaptiveOtosWeight;

        Pose fusedPose = blendPoses(otosPose, encoderPose,
                                     adaptiveOtosWeight, adaptiveEncoderWeight);

        // Apply AprilTag correction if available
        if (useAprilTagCorrection && aprilTagProcessor != null) {
            fusedPose = applyAprilTagCorrection(fusedPose);
        }

        // Update total heading based on pose change
        double headingDelta = fusedPose.getHeading() - previousPose.getHeading();
        // Normalize heading delta
        while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
        while (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;
        totalHeading += headingDelta;

        // Update history
        previousPose = fusedPose;
        currentPose = fusedPose;
        previousTimeNanos = System.nanoTime();

        return fusedPose;
    }

    @Override
    public Pose getVelocity() {
        // Get velocities from sensors
        Pose otosVel = otosLocalizer.getVelocity();
        Pose encoderVel = encoderLocalizer != null ? encoderLocalizer.getVelocity() : otosVel;

        // Blend velocities based on reliability
        double adaptiveOtosWeight = otosWeight * otosReliability;
        double adaptiveEncoderWeight = 1.0 - adaptiveOtosWeight;

        return blendPoses(otosVel, encoderVel, adaptiveOtosWeight, adaptiveEncoderWeight);
    }

    @Override
    public void setStartPose(Pose pose) {
        otosLocalizer.setStartPose(pose);
        if (encoderLocalizer != null) {
            encoderLocalizer.setStartPose(pose);
        }
        startPose = pose;
        previousPose = pose;
        currentPose = pose;
        totalHeading = pose.getHeading();
    }

    @Override
    public void update() {
        otosLocalizer.update();
        if (encoderLocalizer != null) {
            encoderLocalizer.update();
        }
    }

    @Override
    public void setPose(Pose pose) {
        previousPose = currentPose;
        otosLocalizer.setPose(pose);
        if (encoderLocalizer != null) {
            encoderLocalizer.setPose(pose);
        }
        currentPose = pose;
    }

    @Override
    public Vector getVelocityVector() {
        Pose vel = getVelocity();
        return new Vector(vel.getX(), vel.getY());
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        // Return from encoder localizer if available, otherwise from OTOS
        if (encoderLocalizer != null) {
            return encoderLocalizer.getForwardMultiplier();
        }
        return otosLocalizer.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        // Return from encoder localizer if available, otherwise from OTOS
        if (encoderLocalizer != null) {
            return encoderLocalizer.getLateralMultiplier();
        }
        return otosLocalizer.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        // Return from encoder localizer if available, otherwise from OTOS
        if (encoderLocalizer != null) {
            return encoderLocalizer.getTurningMultiplier();
        }
        return otosLocalizer.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {
        otosLocalizer.resetIMU();
        if (encoderLocalizer != null) {
            encoderLocalizer.resetIMU();
        }
        totalHeading = 0.0;
    }

    @Override
    public double getIMUHeading() {
        // Prefer OTOS IMU if available
        return otosLocalizer.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        Pose pose = getPose();
        return Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading());
    }

    /**
     * Detect outliers in OTOS readings and adjust reliability.
     */
    private void detectOutliers(Pose currentPose) {
        long currentTimeNanos = System.nanoTime();
        double deltaTime = (currentTimeNanos - previousTimeNanos) / 1e9;

        if (deltaTime < 0.001) return; // Skip if too soon

        // Calculate position jump
        double dx = currentPose.getX() - previousPose.getX();
        double dy = currentPose.getY() - previousPose.getY();
        double positionJump = Math.sqrt(dx * dx + dy * dy);

        // Calculate implied velocity
        double impliedVelocity = positionJump / deltaTime;

        // Check for outliers
        boolean isOutlier = false;

        if (positionJump > MAX_POSITION_JUMP) {
            isOutlier = true;
        }

        if (impliedVelocity > MAX_VELOCITY_JUMP) {
            isOutlier = true;
        }

        // Update reliability
        if (isOutlier) {
            // Reduce trust in OTOS
            otosReliability *= RELIABILITY_DECAY;
            otosReliability = Math.max(0.1, otosReliability); // Never fully distrust
        } else {
            // Gradually restore trust
            otosReliability += RELIABILITY_RECOVERY * (1.0 - otosReliability);
            otosReliability = Math.min(1.0, otosReliability);
        }
    }

    /**
     * Apply AprilTag correction to the fused pose.
     */
    private Pose applyAprilTagCorrection(Pose fusedPose) {
        if (!aprilTagCorrectionEnabled || aprilTagProcessor == null) {
            return fusedPose;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections.isEmpty()) {
            return fusedPose;
        }

        // Find the detection with highest confidence
        AprilTagDetection bestDetection = null;
        double bestConfidence = 0.0;

        for (AprilTagDetection detection : detections) {
            // Calculate confidence based on detection quality
            double confidence = calculateAprilTagConfidence(detection);

            if (confidence > bestConfidence && confidence > APRILTAG_CONFIDENCE_THRESHOLD) {
                bestConfidence = confidence;
                bestDetection = detection;
            }
        }

        if (bestDetection == null) {
            return fusedPose;
        }

        // Get robot pose from AprilTag detection
        Pose aprilTagPose = calculateRobotPoseFromTag(bestDetection);

        // Blend with current pose
        return blendPoses(fusedPose, aprilTagPose,
                         1.0 - APRILTAG_CORRECTION_WEIGHT,
                         APRILTAG_CORRECTION_WEIGHT);
    }

    /**
     * Calculate confidence in an AprilTag detection.
     */
    private double calculateAprilTagConfidence(AprilTagDetection detection) {
        // Factors affecting confidence:
        // 1. Decision margin (higher is better)
        // 2. Distance (closer is better)
        // 3. Angle (more perpendicular is better)

        double marginScore = Math.min(1.0, detection.decisionMargin / 100.0);

        // Distance score (best at 24-48 inches)
        double range = detection.ftcPose.range;
        double distanceScore = 1.0;
        if (range < 12 || range > 72) {
            distanceScore = 0.5;
        } else if (range < 24 || range > 48) {
            distanceScore = 0.8;
        }

        // Angle score (best when perpendicular)
        double yaw = Math.abs(detection.ftcPose.yaw);
        double angleScore = Math.cos(Math.toRadians(yaw));

        return marginScore * distanceScore * angleScore;
    }

    /**
     * Calculate robot pose from AprilTag detection.
     *
     * NOTE: This is a simplified implementation. You'll need to:
     * 1. Know the AprilTag's field position
     * 2. Transform from camera frame to robot frame
     * 3. Transform from tag frame to field frame
     */
    private Pose calculateRobotPoseFromTag(AprilTagDetection detection) {
        // TODO: Implement proper AprilTag-to-field-pose transformation
        // This requires:
        // - Known AprilTag field positions
        // - Camera-to-robot transformation
        // - Tag-to-field transformation

        // Placeholder: return current pose (no correction)
        return previousPose;
    }

    /**
     * Blend two poses with given weights.
     */
    private Pose blendPoses(Pose pose1, Pose pose2, double weight1, double weight2) {
        double x = pose1.getX() * weight1 + pose2.getX() * weight2;
        double y = pose1.getY() * weight1 + pose2.getY() * weight2;

        // Handle angle blending carefully to avoid wrap-around issues
        double h1 = pose1.getHeading();
        double h2 = pose2.getHeading();

        // Normalize angle difference
        double angleDiff = h2 - h1;
        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

        double h = h1 + angleDiff * weight2;

        return new Pose(x, y, h);
    }

    /**
     * Get current OTOS reliability (0.0 to 1.0).
     */
    public double getOtosReliability() {
        return otosReliability;
    }

    /**
     * Get current fusion weights being used.
     */
    public String getFusionStatus() {
        double adaptiveOtosWeight = otosWeight * otosReliability;
        return String.format("OTOS: %.2f%%, Encoders: %.2f%%, Reliability: %.2f%%",
                           adaptiveOtosWeight * 100,
                           (1.0 - adaptiveOtosWeight) * 100,
                           otosReliability * 100);
    }
}
