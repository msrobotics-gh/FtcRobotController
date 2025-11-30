package org.firstinspires.ftc.teamcode.pedroPathing.asymDrive;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Drive encoder localizer for asymmetric mecanum drivetrains.
 *
 * Uses the 4 drive motor encoders to estimate robot position.
 * Supports different strafe multipliers for left vs right strafing.
 */
public class AsymDriveEncoderLocalizer implements Localizer {

    private final DcMotor leftFront, leftRear, rightFront, rightRear;
    private final AsymDriveEncoderConstants constants;

    // Encoder positions
    private double lfPrev, lrPrev, rfPrev, rrPrev;

    // Current pose estimate
    private Pose pose = new Pose();
    private Pose velocity = new Pose();
    private Pose startPose = new Pose();

    // Total heading for tracking
    private double totalHeading = 0;

    // Timing for velocity calculation
    private long previousTimeNanos = System.nanoTime();

    public AsymDriveEncoderLocalizer(HardwareMap hardwareMap, AsymDriveEncoderConstants constants) {
        this.constants = constants;

        leftFront = hardwareMap.get(DcMotor.class, constants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotor.class, constants.leftRearMotorName);
        rightFront = hardwareMap.get(DcMotor.class, constants.rightFrontMotorName);
        rightRear = hardwareMap.get(DcMotor.class, constants.rightRearMotorName);

        // Reset encoders
        resetEncoders();
    }

    private void resetEncoders() {
        lfPrev = getEncoderPosition(leftFront, constants.leftFrontEncoderDirection);
        lrPrev = getEncoderPosition(leftRear, constants.leftRearEncoderDirection);
        rfPrev = getEncoderPosition(rightFront, constants.rightFrontEncoderDirection);
        rrPrev = getEncoderPosition(rightRear, constants.rightRearEncoderDirection);
    }

    private double getEncoderPosition(DcMotor motor, double direction) {
        return motor.getCurrentPosition() * direction;
    }

    @Override
    public void update() {
        // Get current encoder positions
        double lfCurr = getEncoderPosition(leftFront, constants.leftFrontEncoderDirection);
        double lrCurr = getEncoderPosition(leftRear, constants.leftRearEncoderDirection);
        double rfCurr = getEncoderPosition(rightFront, constants.rightFrontEncoderDirection);
        double rrCurr = getEncoderPosition(rightRear, constants.rightRearEncoderDirection);

        // Calculate deltas
        double lfDelta = lfCurr - lfPrev;
        double lrDelta = lrCurr - lrPrev;
        double rfDelta = rfCurr - rfPrev;
        double rrDelta = rrCurr - rrPrev;

        // Update previous positions
        lfPrev = lfCurr;
        lrPrev = lrCurr;
        rfPrev = rfCurr;
        rrPrev = rrCurr;

        // Mecanum kinematics (standard 4-wheel inverse)
        // Forward: average of all 4 wheels
        double forwardTicks = (lfDelta + lrDelta + rfDelta + rrDelta) / 4.0;

        // Strafe: mecanum diagonal pattern
        // Left strafe: LF-, LR+, RF+, RR-
        // Right strafe: LF+, LR-, RF-, RR+
        double strafeTicks = (-lfDelta + lrDelta + rfDelta - rrDelta) / 4.0;

        // Turn: difference between left and right sides
        double leftAvg = (lfDelta + lrDelta) / 2.0;
        double rightAvg = (rfDelta + rrDelta) / 2.0;
        double turnTicks = (rightAvg - leftAvg) / 2.0;

        // Convert to inches
        double forwardInches = forwardTicks * constants.forwardTicksToInches;

        // Use asymmetric strafe multiplier based on strafe direction
        double strafeMultiplier = constants.getStrafeTicksToInches(strafeTicks);
        double strafeInches = strafeTicks * strafeMultiplier;

        // Convert turn to radians using track width
        double turnRadians = turnTicks * constants.turnTicksToInches / (constants.robot_Width / 2.0);

        // Calculate time delta for velocity
        long currentTimeNanos = System.nanoTime();
        double deltaTime = (currentTimeNanos - previousTimeNanos) / 1e9;
        previousTimeNanos = currentTimeNanos;

        // Update total heading
        totalHeading += turnRadians;

        // Update pose using robot-centric motion transformed to field coordinates
        double heading = pose.getHeading();
        double avgHeading = heading + turnRadians / 2.0; // Use midpoint heading for arc approximation

        double cos = Math.cos(avgHeading);
        double sin = Math.sin(avgHeading);

        // Transform robot-centric motion to field coordinates
        double dx = forwardInches * cos - strafeInches * sin;
        double dy = forwardInches * sin + strafeInches * cos;

        pose = new Pose(
                pose.getX() + dx,
                pose.getY() + dy,
                normalizeAngle(pose.getHeading() + turnRadians)
        );

        // Calculate velocity
        if (deltaTime > 0.001) {
            velocity = new Pose(
                    dx / deltaTime,
                    dy / deltaTime,
                    turnRadians / deltaTime
            );
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    @Override
    public Pose getPose() {
        return pose;
    }

    @Override
    public Pose getVelocity() {
        return velocity;
    }

    @Override
    public Vector getVelocityVector() {
        return new Vector(velocity.getX(), velocity.getY());
    }

    @Override
    public void setStartPose(Pose startPose) {
        this.startPose = startPose;
        this.pose = startPose;
        resetEncoders();
    }

    @Override
    public void setPose(Pose pose) {
        this.pose = pose;
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return constants.forwardTicksToInches;
    }

    @Override
    public double getLateralMultiplier() {
        // Return average of left/right strafe multipliers
        return (constants.strafeLeftTicksToInches + constants.strafeRightTicksToInches) / 2.0;
    }

    @Override
    public double getTurningMultiplier() {
        return constants.turnTicksToInches;
    }

    @Override
    public void resetIMU() throws InterruptedException {
        // No IMU in drive encoder localizer
        totalHeading = 0;
    }

    @Override
    public double getIMUHeading() {
        // No IMU, return calculated heading
        return pose.getHeading();
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading());
    }
}