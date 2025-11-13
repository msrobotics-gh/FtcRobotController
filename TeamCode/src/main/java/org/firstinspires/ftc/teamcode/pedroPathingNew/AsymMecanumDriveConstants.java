package org.firstinspires.ftc.teamcode.pedroPathingNew;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Configuration holder for the asymmetric mecanum drivetrain.
 *
 * === SETUP GUIDE ===
 *
 * 1. MOTOR HARDWARE NAMES
 *    - Check your Robot Configuration in the Driver Station
 *    - Update motor names to match your hardware map (default: "front_left", "back_left", etc.)
 *
 * 2. MOTOR DIRECTIONS
 *    - Test drive forward with all motors set to FORWARD
 *    - Robot should move forward in a straight line
 *    - If it veers or spins, reverse individual motors until it drives straight
 *    - Common configuration: left motors FORWARD or REVERSE, right motors opposite
 *
 * 3. GEOMETRY MEASUREMENTS (CRITICAL FOR ACCURATE TURNING)
 *    - All measurements should be in METERS
 *    - Measure from wheel contact patch centers (where wheels touch the ground)
 *
 *    halfLengthX: Half the distance from FRONT axle to REAR axle
 *                 Measure front-to-rear wheelbase, then divide by 2
 *                 Example: If wheelbase is 36cm, halfLengthX = 0.18m
 *
 *    halfWidthFront: Half the distance between LEFT FRONT and RIGHT FRONT wheels
 *                    Measure left-to-right track width at FRONT, then divide by 2
 *                    Example: If front track is 30cm, halfWidthFront = 0.15m
 *
 *    halfWidthRear: Half the distance between LEFT REAR and RIGHT REAR wheels
 *                   Measure left-to-right track width at REAR, then divide by 2
 *                   Example: If rear track is 44cm, halfWidthRear = 0.22m
 *
 *    NOTE: For symmetric mecanum, halfWidthFront should equal halfWidthRear
 *          For asymmetric (your case), these can differ
 *
 * 4. WHEEL SPECIFICATIONS
 *    wheelRadiusMeters: Radius of your mecanum wheels in meters
 *                       Common sizes: 75mm (0.0375m), 100mm (0.05m), 4" (0.0508m)
 *
 * 5. POWER LIMITS
 *    maxPower: Maximum power multiplier (0.0 to 1.0)
 *              Reduce if robot is too aggressive or wheels slip
 *              Default 1.0 uses full motor power
 *
 * 6. TELEOP BEHAVIOR
 *    useBrakeInTeleOp: true = motors brake when stopped (more control)
 *                      false = motors coast when stopped (easier to push)
 */
public class AsymMecanumDriveConstants {

    // --- Hardware names ---
    public final String lfName, lrName, rfName, rrName;
    public final DcMotorSimple.Direction lfDir, lrDir, rfDir, rrDir;

    // --- Geometry (meters) ---
    public final double halfLengthX;
    public final double halfWidthFront;
    public final double halfWidthRear;

    // --- Wheel & control ---
    public final double wheelRadiusMeters;
    public final double maxPower;
    public final boolean useBrakeInTeleOp;

    public AsymMecanumDriveConstants(
            String lfName, String lrName, String rfName, String rrName,
            DcMotorSimple.Direction lfDir, DcMotorSimple.Direction lrDir,
            DcMotorSimple.Direction rfDir, DcMotorSimple.Direction rrDir,
            double halfLengthX, double halfWidthFront, double halfWidthRear,
            double wheelRadiusMeters, double maxPower, boolean useBrakeInTeleOp) {

        this.lfName = lfName;
        this.lrName = lrName;
        this.rfName = rfName;
        this.rrName = rrName;

        this.lfDir = lfDir;
        this.lrDir = lrDir;
        this.rfDir = rfDir;
        this.rrDir = rrDir;

        this.halfLengthX = halfLengthX;
        this.halfWidthFront = halfWidthFront;
        this.halfWidthRear  = halfWidthRear;

        this.wheelRadiusMeters = wheelRadiusMeters;
        this.maxPower          = Math.max(0.0, Math.min(1.0, maxPower));
        this.useBrakeInTeleOp  = useBrakeInTeleOp;
    }

    /**
     * Default constants - CUSTOMIZE THESE FOR YOUR ROBOT
     * Measure your actual robot and update these values!
     */
    public static AsymMecanumDriveConstants defaults() {
        // === MOTOR HARDWARE NAMES ===
        String leftFrontName = "front_left";
        String leftRearName = "back_left";
        String rightFrontName = "front_right";
        String rightRearName = "back_right";

        // === MOTOR DIRECTIONS ===
        // Adjust these until robot drives straight forward
        DcMotorSimple.Direction leftFrontDir = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction leftRearDir = DcMotorSimple.Direction.REVERSE;
        DcMotorSimple.Direction rightFrontDir = DcMotorSimple.Direction.REVERSE;
        DcMotorSimple.Direction rightRearDir = DcMotorSimple.Direction.REVERSE;

        // === GEOMETRY (meters) ===
        // MEASURE YOUR ROBOT! These are example values.
        double halfLengthX = 0.18;      // Half of front-to-rear wheelbase (36cm wheelbase ÷ 2)
        double halfWidthFront = 0.15;   // Half of front track width (30cm track ÷ 2)
        double halfWidthRear = 0.22;    // Half of rear track width (44cm track ÷ 2)

        // === WHEEL SPECIFICATIONS ===
        double wheelRadiusMeters = 0.0508;  // 4 inch wheels = 0.0508m radius

        // === CONTROL PARAMETERS ===
        double maxPower = 1.0;              // Full power (reduce if too aggressive)
        boolean useBrakeInTeleOp = true;    // Brake mode for better control

        return new AsymMecanumDriveConstants(
                leftFrontName, leftRearName, rightFrontName, rightRearName,
                leftFrontDir, leftRearDir, rightFrontDir, rightRearDir,
                halfLengthX, halfWidthFront, halfWidthRear,
                wheelRadiusMeters, maxPower, useBrakeInTeleOp
        );
    }
}