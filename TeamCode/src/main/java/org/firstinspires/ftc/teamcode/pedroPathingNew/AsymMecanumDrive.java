package org.firstinspires.ftc.teamcode.pedroPathingNew;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Custom asymmetric mecanum drivetrain implementation for Pedro Pathing.
 *
 * This drivetrain supports asymmetric wheel placement (different front/rear track widths)
 * and includes voltage compensation for consistent performance across battery levels.
 */
public class AsymMecanumDrive extends Drivetrain {
    private final HardwareMap hardwareMap;
    private final AsymMecanumDriveConstants constants;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private VoltageSensor voltageSensor;

    private double xVelocity = 0.0;
    private double yVelocity = 0.0;

    /**
     * Creates a new AsymMecanumDrive instance.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param constants Configuration constants for the drivetrain
     */
    public AsymMecanumDrive(HardwareMap hardwareMap, AsymMecanumDriveConstants constants) {
        this.hardwareMap = hardwareMap;
        this.constants = constants;

        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, constants.lfName);
        leftRear = hardwareMap.get(DcMotorEx.class, constants.lrName);
        rightFront = hardwareMap.get(DcMotorEx.class, constants.rfName);
        rightRear = hardwareMap.get(DcMotorEx.class, constants.rrName);

        // Set motor directions
        leftFront.setDirection(constants.lfDir);
        leftRear.setDirection(constants.lrDir);
        rightFront.setDirection(constants.rfDir);
        rightRear.setDirection(constants.rrDir);

        // Get voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Set default voltage compensation values
        this.nominalVoltage = 12.0;
        this.voltageCompensation = true;

        // Set maximum power scaling from constants
        this.maxPowerScaling = constants.maxPower;

        // Calculate movement vectors for asymmetric mecanum
        calculateMovementVectors();
    }

    /**
     * Calculates the moment arm (distance from robot center) for each wheel.
     * These distances are used to properly scale turning contributions in asymmetric mecanum.
     */
    private double radiusFront, radiusRear;

    private void calculateMovementVectors() {
        // For asymmetric mecanum, calculate the distance from each wheel to the robot center
        // This affects the turning moment arm - wheels farther from center contribute more to rotation

        double halfLength = constants.halfLengthX;
        double halfWidthF = constants.halfWidthFront;
        double halfWidthR = constants.halfWidthRear;

        radiusFront = Math.sqrt(halfLength * halfLength + halfWidthF * halfWidthF);
        radiusRear = Math.sqrt(halfLength * halfLength + halfWidthR * halfWidthR);

        // Mecanum wheel force vectors (normalized for 45° rollers)
        // Vector constructor takes (magnitude, theta) where theta is in radians
        vectors = new Vector[4];

        // Left front: 45° (northeast direction)
        vectors[0] = new Vector(1.0, Math.PI / 4);

        // Left rear: 135° (northwest direction)
        vectors[1] = new Vector(1.0, 3 * Math.PI / 4);

        // Right front: -45° or 315° (southeast direction)
        vectors[2] = new Vector(1.0, -Math.PI / 4);

        // Right rear: -135° or 225° (southwest direction)
        vectors[3] = new Vector(1.0, -3 * Math.PI / 4);
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // Combine all power vectors by adding their x and y components
        double xTotal = correctivePower.getXComponent() + pathingPower.getXComponent();
        double yTotal = correctivePower.getYComponent() + pathingPower.getYComponent();

        // Rotate the power vector by the robot's heading to convert from field-centric to robot-centric
        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        double xRobot = xTotal * cos - yTotal * sin;
        double yRobot = xTotal * sin + yTotal * cos;

        // Get turning power from heading vector
        double turn = headingPower.getMagnitude();
        if (headingPower.getXComponent() < 0) {
            turn = -turn; // Turn direction based on x component of heading vector
        }

        // Calculate wheel powers using asymmetric mecanum kinematics
        // Scale turning by distance from center (moment arm) for each wheel
        // Front wheels use radiusFront, rear wheels use radiusRear
        double lfPower = yRobot + xRobot + turn * radiusFront;
        double lrPower = yRobot - xRobot + turn * radiusRear;
        double rfPower = yRobot - xRobot - turn * radiusFront;
        double rrPower = yRobot + xRobot - turn * radiusRear;

        // Find maximum absolute power
        double maxPower = Math.max(
            Math.max(Math.abs(lfPower), Math.abs(lrPower)),
            Math.max(Math.abs(rfPower), Math.abs(rrPower))
        );

        // Normalize if any power exceeds 1.0
        if (maxPower > 1.0) {
            lfPower /= maxPower;
            lrPower /= maxPower;
            rfPower /= maxPower;
            rrPower /= maxPower;
        }

        // Apply maximum power scaling
        lfPower *= maxPowerScaling;
        lrPower *= maxPowerScaling;
        rfPower *= maxPowerScaling;
        rrPower *= maxPowerScaling;

        // Apply voltage compensation if enabled
        if (voltageCompensation) {
            double voltageRatio = nominalVoltage / getVoltage();
            voltageRatio = MathFunctions.clamp(voltageRatio, 0.5, 1.5); // Limit compensation range

            lfPower *= voltageRatio;
            lrPower *= voltageRatio;
            rfPower *= voltageRatio;
            rrPower *= voltageRatio;

            // Re-normalize after voltage compensation
            maxPower = Math.max(
                Math.max(Math.abs(lfPower), Math.abs(lrPower)),
                Math.max(Math.abs(rfPower), Math.abs(rrPower))
            );
            if (maxPower > 1.0) {
                lfPower /= maxPower;
                lrPower /= maxPower;
                rfPower /= maxPower;
                rrPower /= maxPower;
            }
        }

        return new double[] { lfPower, lrPower, rfPower, rrPower };
    }

    @Override
    public void runDrive(double[] drivePowers) {
        if (drivePowers.length < 4) {
            throw new IllegalArgumentException("Drive powers array must have at least 4 elements");
        }

        leftFront.setPower(drivePowers[0]);
        leftRear.setPower(drivePowers[1]);
        rightFront.setPower(drivePowers[2]);
        rightRear.setPower(drivePowers[3]);
    }

    @Override
    public void updateConstants() {
        // Recalculate movement vectors if geometry changed
        calculateMovementVectors();

        // Update max power scaling
        this.maxPowerScaling = constants.maxPower;
    }

    @Override
    public void breakFollowing() {
        // Stop all motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        // Set motors to brake mode
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void startTeleopDrive() {
        startTeleopDrive(constants.useBrakeInTeleOp);
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        DcMotor.ZeroPowerBehavior behavior = brakeMode
            ? DcMotor.ZeroPowerBehavior.BRAKE
            : DcMotor.ZeroPowerBehavior.FLOAT;

        leftFront.setZeroPowerBehavior(behavior);
        leftRear.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightRear.setZeroPowerBehavior(behavior);

        // Set to run without encoder for TeleOp
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public double xVelocity() {
        return xVelocity;
    }

    @Override
    public double yVelocity() {
        return yVelocity;
    }

    @Override
    public void setXVelocity(double xMovement) {
        this.xVelocity = xMovement;
    }

    @Override
    public void setYVelocity(double yMovement) {
        this.yVelocity = yMovement;
    }

    @Override
    public double getVoltage() {
        return voltageSensor != null ? voltageSensor.getVoltage() : nominalVoltage;
    }

    @Override
    public String debugString() {
        return String.format(
            "AsymMecanumDrive[LF:%.2f LR:%.2f RF:%.2f RR:%.2f | V:%.1f/%.1f | xVel:%.2f yVel:%.2f]",
            leftFront.getPower(), leftRear.getPower(), rightFront.getPower(), rightRear.getPower(),
            getVoltage(), nominalVoltage, xVelocity, yVelocity
        );
    }
}
