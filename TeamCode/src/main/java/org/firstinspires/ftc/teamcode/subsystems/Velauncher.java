package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Velauncher Subsystem - Dual Flywheel Velocity Control
 *
 * This subsystem controls two independent flywheel motors (upper and lower) using velocity control
 * with PIDF + feedforward. All tuning parameters are exposed via FTCDashboard for live tuning.
 *
 * TUNING PROCEDURE:
 *
 * 1. INITIAL SETUP
 *    - Set TARGET_RPM_UPPER and TARGET_RPM_LOWER to desired speeds
 *    - Set TPR_UPPER and TPR_LOWER to your motor's ticks per revolution (typically 28 for gobilda)
 *    - Verify motor directions with REVERSE_UPPER and REVERSE_LOWER
 *    - Start with CLOSED_LOOP_ENABLED = false to test open-loop power first
 *
 * 2. OPEN-LOOP BASELINE (CLOSED_LOOP_ENABLED = false)
 *    - Adjust OPEN_LOOP_POWER_UPPER and OPEN_LOOP_POWER_LOWER
 *    - Find the minimum power that reaches approximately your target RPM
 *    - This gives you a baseline for feedforward tuning
 *    - Note: Open-loop is not accurate but helps establish motor characteristics
 *
 * 3. FEEDFORWARD TUNING (Set CLOSED_LOOP_ENABLED = true)
 *
 *    a) Tune kV (Velocity Feedforward) - MOST IMPORTANT
 *       - Formula: kV ≈ open_loop_power / target_velocity_tps
 *       - Example: If 0.7 power gives ~850 RPM (396 tps), then kV ≈ 0.7/396 ≈ 0.00177
 *       - Start with calculated value, then fine-tune in FTCDashboard
 *       - Goal: Motor velocity should track target with minimal oscillation
 *
 *    b) Tune kS (Static Friction Compensation)
 *       - Start at 0.0, gradually increase if motor doesn't start at low speeds
 *       - Typical range: 0.0 to 0.05
 *       - Too high: Motor will overshoot at startup
 *       - Too low: Motor won't overcome static friction
 *
 *    c) Tune kA (Acceleration Feedforward) - Usually 0.0
 *       - Only needed if you have rapid velocity changes
 *       - For constant-speed flywheels, leave at 0.0
 *
 * 4. FEEDBACK TUNING (After feedforward is close)
 *
 *    a) Tune kP (Proportional Gain)
 *       - Start very small (0.00001 to 0.0001)
 *       - Increase until velocity error is minimized
 *       - Too high: Oscillation or instability
 *       - Too low: Slow response, persistent error
 *       - Watch "Upper_err_tps" and "Lower_err_tps" in FTCDashboard
 *
 *    b) Tune kD (Derivative Gain) - Usually 0.0
 *       - Only add if you have overshoot/oscillation after tuning kP
 *       - Start at 0.0, increase gradually if needed
 *       - Helps dampen oscillations
 *
 *    c) Tune kI (Integral Gain) - USE SPARINGLY
 *       - Only add if you have persistent steady-state error
 *       - Start at 0.0, increase very slowly (0.00001 increments)
 *       - Too high: Integral windup, instability, overshoot
 *       - Often better to fix with kV than to rely on kI
 *
 * 5. VALIDATION
 *    - Check "Upper_atSpeed" and "Lower_atSpeed" indicators in FTCDashboard
 *    - Verify error stays within TOL_TPS_UPPER and TOL_TPS_LOWER
 *    - Test with actual game pieces to ensure velocity holds under load
 *    - Adjust TOL_TPS values if needed (default: 50 tps tolerance)
 *
 * 6. ADVANCED TUNING
 *    - VEL_LP_ALPHA: Low-pass filter for velocity measurement (0.0-1.0)
 *      - Lower = more filtering (smoother, slower response)
 *      - Set to -1.0 to disable (default)
 *      - Only use if velocity readings are very noisy
 *
 * MONITORING IN FTCDASHBOARD:
 * - Upper_target_rpm / Lower_target_rpm: Your setpoint
 * - Upper_meas_rpm / Lower_meas_rpm: Actual measured velocity
 * - Upper_err_tps / Lower_err_tps: Error in ticks/second
 * - Upper_power / Lower_power: Commanded motor power
 * - Upper_atSpeed / Lower_atSpeed: Whether within tolerance
 *
 * TYPICAL TUNING ORDER:
 * 1. kV (most important, gets you 80-90% there)
 * 2. kS (if motor doesn't start smoothly)
 * 3. kP (reduce remaining error)
 * 4. kD (if oscillating)
 * 5. kI (last resort for persistent error)
 *
 * TROUBLESHOOTING:
 * - Motor won't reach speed: Increase kV or add kS
 * - Motor oscillates: Reduce kP or add kD
 * - Motor overshoots then settles: Reduce kP, add kD
 * - Persistent steady-state error: Fine-tune kV first, then consider small kI
 * - Unstable/runaway: Reduce all gains, start over with smaller kP
 */

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.net.PortUnreachableException;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.FeedbackElement;
import dev.nextftc.control.feedback.FeedbackType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDElement;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToState;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.delegates.Velocity;
import dev.nextftc.hardware.driving.HolonomicDrivePowers;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.Powerable;
import dev.nextftc.hardware.powerable.SetPower;
@Config

public class Velauncher implements Subsystem {

    // === Target speeds (RPM), kV ===
    // CONSTANTS
    public final static double TARGET_RPM_UPPER = 2157;
    public final static double TARGET_RPM_LOWER = 3442;

    public final static double kV_UPPER = 0.000415;
    public final static double kV_LOWER = 0.000419;


    // === TPR (what does this mean) ===
    public static double TPR_UPPER = 28; // ticks per output revolution (upper motor)
    public static double TPR_LOWER = 28; // ticks per output revolution (lower motor)



    // === Velocity tolerance for "at speed" checks (ticks/sec) ===
    public static double TOL_TPS_UPPER = 50.0;
    public static double TOL_TPS_LOWER = 50.0;

    // === Control enables ===
    public static boolean ENABLED_UPPER = true;
    public static boolean ENABLED_LOWER = true;
    public static boolean CLOSED_LOOP_ENABLED = true;

    // === Open-loop fallback ===
    public static double OPEN_LOOP_POWER_UPPER = 0.7;
    public static double OPEN_LOOP_POWER_LOWER = 0.7;

    // === NextFTC Control gains — Upper motor ===
    public static double kP_UPPER = 0.00002;
    public static double kI_UPPER = 0.0;
    public static double kD_UPPER = 0.0;
    public static double kA_UPPER = 0.0;
    public static double kS_UPPER = 0.02;

    // === NextFTC Control gains — Lower motor ===
    public static double kP_LOWER = 0.00002;
    public static double kI_LOWER = 0.0;
    public static double kD_LOWER = 0.0;

    public static double kA_LOWER = 0.0;
    public static double kS_LOWER = 0.02;

    // === Optional velocity low-pass ===
    public static double VEL_LP_ALPHA = -1.0; // e.g., 0.2 for filtering

    // === Directions ===
    public static boolean REVERSE_UPPER = false;
    public static boolean REVERSE_LOWER = true;

    private ControlSystem ctrlUpper, ctrlLower;

    // Cache current tunables
    private double _kP_UPPER, _kI_UPPER, _kD_UPPER, _kV_UPPER, _kA_UPPER, _kS_UPPER;
    private double _kP_LOWER, _kI_LOWER, _kD_LOWER, _kV_LOWER, _kA_LOWER, _kS_LOWER;
    private double _VEL_LP_ALPHA;

    // Flywheel state control
    private boolean flywheelEnabled = false;

    public static final Velauncher INSTANCE = new Velauncher();
    private Velauncher() { }

    private MotorEx upperMotor = new MotorEx("launch").reversed();
    private MotorEx lowerMotor = new MotorEx("launch2").reversed();

    @Override
    public void initialize(){

        buildControllers();

    }

    //private FeedbackElement pid = new PIDElement(FeedbackType.VELOCITY, coefficients);
    private void buildControllers() {
        ControlSystem bUpper = ControlSystem.builder()
                .velPid(kP_UPPER, kI_UPPER, kD_UPPER)
                .basicFF(kV_UPPER, kA_UPPER, kS_UPPER)
                .build();
        if (VEL_LP_ALPHA >= 0.0 && VEL_LP_ALPHA <= 1.0) {
            //bUpper.lowPass(VEL_LP_ALPHA); // uncomment if supported
        }
        ctrlUpper = bUpper;

        ControlSystem bLower = ControlSystem.builder()
                .velPid(kP_LOWER, kI_LOWER, kD_LOWER)
                .basicFF(kV_LOWER, kA_LOWER, kS_LOWER)
                .build();
        if (VEL_LP_ALPHA >= 0.0 && VEL_LP_ALPHA <= 1.0) {
            //bLower.lowPass(VEL_LP_ALPHA);
        }
        ctrlLower = bLower;

        _kP_UPPER = kP_UPPER; _kI_UPPER = kI_UPPER; _kD_UPPER = kD_UPPER; _kV_UPPER = kV_UPPER; _kA_UPPER = kA_UPPER; _kS_UPPER = kS_UPPER;
        _kP_LOWER = kP_LOWER; _kI_LOWER = kI_LOWER; _kD_LOWER = kD_LOWER; _kV_LOWER = kV_LOWER; _kA_LOWER = kA_LOWER; _kS_LOWER = kS_LOWER;
        _VEL_LP_ALPHA = VEL_LP_ALPHA;
    }

    private boolean gainsChanged() {
        return _kP_UPPER != kP_UPPER || _kI_UPPER != kI_UPPER || _kD_UPPER != kD_UPPER || _kV_UPPER != kV_UPPER || _kA_UPPER != kA_UPPER || _kS_UPPER != kS_UPPER
                || _kP_LOWER != kP_LOWER || _kI_LOWER != kI_LOWER || _kD_LOWER != kD_LOWER || _kV_LOWER != kV_LOWER || _kA_LOWER != kA_LOWER || _kS_LOWER != kS_LOWER
                || _VEL_LP_ALPHA != VEL_LP_ALPHA;
    }

    public  Command velaunch = new LambdaCommand()
            .setStart(() -> {
                flywheelEnabled = true;
            })
            .setIsDone(() -> true);

    public  Command unvelaunch = new LambdaCommand()
            .setStart(() -> {
                flywheelEnabled = false;
            })
            .setIsDone(() -> true);




    @Override
    public void periodic() {
        // Rebuild controllers if gains changed via FTCDashboard
        if (gainsChanged()) buildControllers();

        double powerUpper, powerLower;

        // Always use current target values from FTCDashboard
        final double targetTpsUpper = rpmToTps(TARGET_RPM_UPPER, TPR_UPPER);
        final double targetTpsLower = rpmToTps(TARGET_RPM_LOWER, TPR_LOWER);

        // Set goals based on flywheel state
        if (flywheelEnabled) {
            ctrlUpper.setGoal(new KineticState(0.0, targetTpsUpper, 0.0));
            ctrlLower.setGoal(new KineticState(0.0, targetTpsLower, 0.0));
        } else {
            ctrlUpper.setGoal(new KineticState(0.0, 0.0, 0.0));
            ctrlLower.setGoal(new KineticState(0.0, 0.0, 0.0));
        }

        if (CLOSED_LOOP_ENABLED) {
            powerUpper = ctrlUpper.calculate(upperMotor.getState());
            powerLower = ctrlLower.calculate(lowerMotor.getState());
        } else {
            powerUpper = OPEN_LOOP_POWER_UPPER;
            powerLower = OPEN_LOOP_POWER_LOWER;
        }

        upperMotor.setPower(powerUpper);
        lowerMotor.setPower(powerLower);

        final double measTpsUpper = Math.abs(upperMotor.getVelocity());
        final double measTpsLower = Math.abs(lowerMotor.getVelocity());
        final double measRpmUpper = tpsToRpm(measTpsUpper, TPR_UPPER);
        final double measRpmLower = tpsToRpm(measTpsLower, TPR_LOWER);

        final double errUpper = targetTpsUpper - measTpsUpper;
        final double errLower = targetTpsLower - measTpsLower;

        final boolean atSpeedUpper = Math.abs(errUpper) <= TOL_TPS_UPPER;
        final boolean atSpeedLower = Math.abs(errLower) <= TOL_TPS_LOWER;

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("MEASURED UPPER RPM", measRpmUpper);
        packet.put("MEASURED LOWER", measRpmLower);

        packet.put("---", "---");

        packet.put("Upper_target_tps", targetTpsUpper);
        packet.put("Upper_meas_tps", measTpsUpper);
        packet.put("Upper_target_rpm", TARGET_RPM_UPPER);
        packet.put("Upper_meas_rpm", measRpmUpper);
        packet.put("Upper_err_tps", errUpper);
        packet.put("Upper_power", powerUpper);
        packet.put("Upper_atSpeed", atSpeedUpper);

        packet.put("Lower_target_tps", targetTpsLower);
        packet.put("Lower_meas_tps", measTpsLower);
        packet.put("Lower_target_rpm", TARGET_RPM_LOWER);
        packet.put("Lower_meas_rpm", measRpmLower);
        packet.put("Lower_err_tps", errLower);
        packet.put("Lower_power", powerLower);
        packet.put("Lower_atSpeed", atSpeedLower);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        Telemetry telemetry = ActiveOpMode.telemetry();
        telemetry.addData("Upper RPM", "%.0f / %.0f (err=%.0f) power=%.2f %s",
                measRpmUpper, TARGET_RPM_UPPER, tpsToRpm(errUpper, TPR_UPPER), powerUpper, atSpeedUpper ? "✓" : "✗");
        telemetry.addData("Lower RPM", "%.0f / %.0f (err=%.0f) power=%.2f %s",
                measRpmLower, TARGET_RPM_LOWER, tpsToRpm(errLower, TPR_LOWER), powerLower, atSpeedLower ? "✓" : "✗");
        telemetry.addData("Closed Loop", CLOSED_LOOP_ENABLED);
        telemetry.update();
    }
    private static double rpmToTps(double rpm, double tpr) {
        return rpm * tpr / 60.0;
    }

    private static double tpsToRpm(double tps, double tpr) {
        return (tps * 60.0) / tpr;
    }
}




/*
lower measured (actually upper) is 3500rpm
upper measured (actually lower) is 2500rpm
*/