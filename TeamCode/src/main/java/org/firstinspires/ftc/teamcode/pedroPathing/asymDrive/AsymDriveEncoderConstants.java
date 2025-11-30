package org.firstinspires.ftc.teamcode.pedroPathing.asymDrive;

import com.pedropathing.ftc.localization.Encoder;

/**
 * Drive encoder constants for asymmetric mecanum drives.
 *
 * Unlike the standard DriveEncoderConstants, this supports different
 * strafe multipliers for left vs right strafing to account for
 * mechanical asymmetry in the drivetrain.
 */
public class AsymDriveEncoderConstants {

    public double forwardTicksToInches = 1;
    public double strafeLeftTicksToInches = 1;   // Left strafe multiplier
    public double strafeRightTicksToInches = 1;  // Right strafe multiplier
    public double turnTicksToInches = 1;

    public double robot_Width = 1;
    public double robot_Length = 1;

    // Default directions matching AsymMecanumDriveConstants defaults
    public double leftFrontEncoderDirection = Encoder.FORWARD;
    public double rightFrontEncoderDirection = Encoder.REVERSE;
    public double leftRearEncoderDirection = Encoder.REVERSE;
    public double rightRearEncoderDirection = Encoder.REVERSE;

    // Default motor names matching AsymMecanumDriveConstants defaults
    public String leftFrontMotorName = "front_left";
    public String leftRearMotorName = "back_left";
    public String rightFrontMotorName = "front_right";
    public String rightRearMotorName = "back_right";

    public AsymDriveEncoderConstants forwardTicksToInches(double forwardTicksToInches) {
        this.forwardTicksToInches = forwardTicksToInches;
        return this;
    }

    /**
     * Set symmetric strafe multiplier (same for left and right).
     */
    public AsymDriveEncoderConstants strafeTicksToInches(double strafeTicksToInches) {
        this.strafeLeftTicksToInches = strafeTicksToInches;
        this.strafeRightTicksToInches = strafeTicksToInches;
        return this;
    }

    /**
     * Set asymmetric strafe multipliers.
     */
    public AsymDriveEncoderConstants strafeTicksToInches(double leftTicksToInches, double rightTicksToInches) {
        this.strafeLeftTicksToInches = leftTicksToInches;
        this.strafeRightTicksToInches = rightTicksToInches;
        return this;
    }

    public AsymDriveEncoderConstants strafeLeftTicksToInches(double strafeLeftTicksToInches) {
        this.strafeLeftTicksToInches = strafeLeftTicksToInches;
        return this;
    }

    public AsymDriveEncoderConstants strafeRightTicksToInches(double strafeRightTicksToInches) {
        this.strafeRightTicksToInches = strafeRightTicksToInches;
        return this;
    }

    public AsymDriveEncoderConstants turnTicksToInches(double turnTicksToInches) {
        this.turnTicksToInches = turnTicksToInches;
        return this;
    }

    public AsymDriveEncoderConstants robotWidth(double robot_Width) {
        this.robot_Width = robot_Width;
        return this;
    }

    public AsymDriveEncoderConstants robotLength(double robot_Length) {
        this.robot_Length = robot_Length;
        return this;
    }

    public AsymDriveEncoderConstants leftFrontEncoderDirection(double leftFrontEncoderDirection) {
        this.leftFrontEncoderDirection = leftFrontEncoderDirection;
        return this;
    }

    public AsymDriveEncoderConstants rightFrontEncoderDirection(double rightFrontEncoderDirection) {
        this.rightFrontEncoderDirection = rightFrontEncoderDirection;
        return this;
    }

    public AsymDriveEncoderConstants leftRearEncoderDirection(double leftRearEncoderDirection) {
        this.leftRearEncoderDirection = leftRearEncoderDirection;
        return this;
    }

    public AsymDriveEncoderConstants rightRearEncoderDirection(double rightRearEncoderDirection) {
        this.rightRearEncoderDirection = rightRearEncoderDirection;
        return this;
    }

    public AsymDriveEncoderConstants leftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    public AsymDriveEncoderConstants leftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public AsymDriveEncoderConstants rightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public AsymDriveEncoderConstants rightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
        return this;
    }

    /**
     * Get the appropriate strafe multiplier based on strafe direction.
     *
     * @param strafeDirection Positive = right, Negative = left
     * @return The appropriate ticks-to-inches multiplier
     */
    public double getStrafeTicksToInches(double strafeDirection) {
        if (strafeDirection >= 0) {
            return strafeRightTicksToInches;
        } else {
            return strafeLeftTicksToInches;
        }
    }

    public static AsymDriveEncoderConstants defaults() {
        return new AsymDriveEncoderConstants();
    }
}