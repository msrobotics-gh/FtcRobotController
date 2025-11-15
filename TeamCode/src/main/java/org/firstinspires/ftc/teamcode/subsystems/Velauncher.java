package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Velauncher Subsystem - Dual Flywheel Velocity Control with Voltage Compensation
 *
 * This subsystem controls two independent flywheel motors (upper and lower) using velocity control
 * with PIDF + feedforward. Includes automatic battery voltage compensation to maintain consistent
 * performance regardless of battery charge level. All tuning parameters are exposed via FTCDashboard.
 *
 * === VOLTAGE COMPENSATION OVERVIEW ===
 *
 * Battery voltage drops from ~13.5V (full) to ~11V (discharged) during matches. Without compensation,
 * motor RPM decreases as battery drains, requiring constant kV retuning.
 *
 * This implementation automatically scales feedforward gains (kV, kS) based on battery voltage:
 *   - Tune gains ONCE at full battery
 *   - System automatically compensates as battery drains
 *   - Consistent RPM throughout entire match
 *   - No more mid-match kV adjustments needed!
 *
 * === TUNING PROCEDURE ===
 *
 * 1. INITIAL SETUP
 *    - Fully charge battery to ~13.5V
 *    - Set TARGET_RPM_UPPER and TARGET_RPM_LOWER to desired speeds
 *    - Set TPR_UPPER and TPR_LOWER to your motor's ticks per revolution (typically 28 for GoBilda)
 *    - Verify motor directions with REVERSE_UPPER and REVERSE_LOWER
 *    - Set VOLTAGE_COMPENSATION_ENABLED = true (recommended)
 *    - Start with CLOSED_LOOP_ENABLED = false to test open-loop power first
 *
 * 2. OPEN-LOOP BASELINE (CLOSED_LOOP_ENABLED = false)
 *    - Adjust OPEN_LOOP_POWER_UPPER and OPEN_LOOP_POWER_LOWER
 *    - Find the minimum power that reaches approximately your target RPM
 *    - This gives you a baseline for feedforward tuning
 *    - Note the battery voltage displayed in telemetry
 *    - Note: Open-loop is not accurate but helps establish motor characteristics
 *
 * 3. FEEDFORWARD TUNING (Set CLOSED_LOOP_ENABLED = true)
 *
 *    IMPORTANT: Tune with FULLY CHARGED battery (~13.5V)
 *    - The battery voltage when you tune will be saved as NOMINAL_VOLTAGE
 *    - All future voltage compensation references this baseline voltage
 *
 *    a) Tune kV_NOMINAL (Velocity Feedforward) - MOST IMPORTANT
 *       - Formula: kV ≈ open_loop_power / target_velocity_tps
 *       - Example: If 0.7 power gives ~850 RPM (396 tps), then kV ≈ 0.7/396 ≈ 0.00177
 *       - Start with calculated value, then fine-tune in FTCDashboard
 *       - Goal: Motor velocity should track target with minimal oscillation
 *       - Watch "kV_compensated" in telemetry to see voltage-adjusted value
 *
 *    b) Tune kS_NOMINAL (Static Friction Compensation)
 *       - Start at 0.0, gradually increase if motor doesn't start at low speeds
 *       - Typical range: 0.0 to 0.05
 *       - Too high: Motor will overshoot at startup
 *       - Too low: Motor won't overcome static friction
 *
 *    c) Set NOMINAL_VOLTAGE
 *       - After tuning kV and kS, note the current battery voltage in telemetry
 *       - Set NOMINAL_VOLTAGE to this value (e.g., 13.2V)
 *       - This locks in your tuning reference point
 *
 *    d) Tune kA (Acceleration Feedforward) - Usually 0.0
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
 *       - With good voltage compensation, kI is rarely needed
 *
 * 5. VOLTAGE COMPENSATION VALIDATION
 *    - After tuning, test with partially drained battery (~11.5-12V)
 *    - RPM should remain consistent despite lower voltage
 *    - Check "Voltage_Ratio" in telemetry - should be >1.0 with drained battery
 *    - Check "kV_compensated" - should be higher than kV_NOMINAL with drained battery
 *    - Motor power should automatically increase to compensate for voltage drop
 *    - If RPM drops with lower voltage, verify VOLTAGE_COMPENSATION_ENABLED = true
 *
 * 6. FINAL VALIDATION
 *    - Check "Upper_atSpeed" and "Lower_atSpeed" indicators in FTCDashboard
 *    - Verify error stays within TOL_TPS_UPPER and TOL_TPS_LOWER
 *    - Test with actual game pieces to ensure velocity holds under load
 *    - Test throughout a full battery cycle (13.5V → 11V)
 *    - Adjust TOL_TPS values if needed (default: 50 tps tolerance)
 *
 * 7. ADVANCED TUNING
 *    - VEL_LP_ALPHA: Low-pass filter for velocity measurement (0.0-1.0)
 *      - Lower = more filtering (smoother, slower response)
 *      - Set to -1.0 to disable (default)
 *      - Only use if velocity readings are very noisy
 *
 * === MONITORING IN FTCDASHBOARD ===
 * Core Metrics:
 * - Upper_target_rpm / Lower_target_rpm: Your setpoint
 * - Upper_meas_rpm / Lower_meas_rpm: Actual measured velocity
 * - Upper_err_tps / Lower_err_tps: Error in ticks/second
 * - Upper_power / Lower_power: Commanded motor power
 * - Upper_atSpeed / Lower_atSpeed: Whether within tolerance
 *
 * Voltage Compensation Metrics:
 * - Battery_Voltage: Current battery voltage (V)
 * - Voltage_Ratio: NOMINAL_VOLTAGE / Battery_Voltage (>1.0 = compensation active)
 * - kV_Upper_Compensated / kV_Lower_Compensated: Voltage-adjusted feedforward gains
 * - kS_Upper_Compensated / kS_Lower_Compensated: Voltage-adjusted static friction
 *
 * === TYPICAL TUNING ORDER ===
 * 1. kV_NOMINAL (most important, gets you 80-90% there)
 * 2. kS_NOMINAL (if motor doesn't start smoothly)
 * 3. Set NOMINAL_VOLTAGE to current battery voltage
 * 4. kP (reduce remaining error)
 * 5. kD (if oscillating)
 * 6. kI (last resort, usually not needed with voltage compensation)
 *
 * === TROUBLESHOOTING ===
 * General:
 * - Motor won't reach speed: Increase kV_NOMINAL or add kS_NOMINAL
 * - Motor oscillates: Reduce kP or add kD
 * - Motor overshoots then settles: Reduce kP, add kD
 * - Persistent steady-state error: Fine-tune kV_NOMINAL first, then consider small kI
 * - Unstable/runaway: Reduce all gains, start over with smaller kP
 *
 * Voltage Compensation Specific:
 * - RPM drops as battery drains: Verify VOLTAGE_COMPENSATION_ENABLED = true
 * - RPM drops as battery drains (compensation enabled): Check NOMINAL_VOLTAGE matches tuning voltage
 * - Voltage_Ratio stuck at 1.0: Battery voltage reading may be incorrect, check voltage sensor
 * - Motor power hits 1.0 with low battery: Battery too drained, gains too aggressive, or target RPM too high
 * - Inconsistent behavior: Retune kV_NOMINAL with fresh battery, ensure NOMINAL_VOLTAGE is set correctly
 *
 * === HOW VOLTAGE COMPENSATION WORKS ===
 *
 * Physics: Motor speed is proportional to applied voltage
 * - At 13V: power command of 0.8 applies 10.4V to motor
 * - At 11V: same 0.8 power only applies 8.8V to motor (16% less!)
 *
 * Solution: Scale feedforward gains by voltage ratio
 * - kV_compensated = kV_NOMINAL × (NOMINAL_VOLTAGE / current_voltage)
 * - At 11V battery: kV_compensated = kV_NOMINAL × (13/11) = 1.18× higher
 * - Higher gain → higher power command → same actual voltage to motor
 *
 * Result: Consistent motor voltage and RPM regardless of battery charge!
 */

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.FeedbackElement;
import dev.nextftc.control.feedback.FeedbackType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDElement;
import dev.nextftc.control.filters.LowPassFilter;
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

    // === Target speeds (RPM) ===
    public static double TARGET_RPM_UPPER = 3500;
    public static double TARGET_RPM_LOWER = 2500;

    // === Voltage Compensation ===
    public static boolean VOLTAGE_COMPENSATION_ENABLED = true;
    public static double NOMINAL_VOLTAGE = 12.92; // Battery voltage when kV was tuned (set after tuning!)
    public static int VOLTAGE_COMP_REBUILD_CYCLES = 50; // Rebuild controllers every N cycles to update voltage compensation

    // === Feedforward Gains (Nominal - tuned at NOMINAL_VOLTAGE) ===
    public static double kV_UPPER_NOMINAL = 0.0003587;
    public static double kV_LOWER_NOMINAL = 0.00038;

    // === TPR (Ticks Per Revolution) ===
    public static double TPR_UPPER = 28; // ticks per output revolution (upper motor)
    public static double TPR_LOWER = 28; // ticks per output revolution (lower motor)



    // === Velocity tolerance for "at speed" checks (ticks/sec) ===
    public static double TOL_TPS_UPPER = 50.0;
    public static double TOL_TPS_LOWER = 50.0;

    // === Control enables ===
    public static boolean CLOSED_LOOP_ENABLED = true;

    // === Open-loop fallback ===
    public static double OPEN_LOOP_POWER_UPPER = 0.7;
    public static double OPEN_LOOP_POWER_LOWER = 0.7;

    // === Feedback Gains (PID) — Upper motor ===
    public static double kP_UPPER = 0.00002;
    public static double kI_UPPER = 0.0;
    public static double kD_UPPER = 0.0;

    // === Feedforward Gains (Nominal) — Upper motor ===
    public static double kA_UPPER = 0.0;
    public static double kS_UPPER_NOMINAL = 0.02;

    // === Feedback Gains (PID) — Lower motor ===
    public static double kP_LOWER = 0.00002;
    public static double kI_LOWER = 0.0;
    public static double kD_LOWER = 0.0;

    // === Feedforward Gains (Nominal) — Lower motor ===
    public static double kA_LOWER = 0.0;
    public static double kS_LOWER_NOMINAL = 0.02;

    // === Optional velocity low-pass ===
    public static double VEL_LP_ALPHA = -1.0; // e.g., 0.2 for filtering

    // === Directions ===
    public static boolean REVERSE_UPPER = false;
    public static boolean REVERSE_LOWER = true;

    private ControlSystem ctrlUpper, ctrlLower;
    private VoltageSensor voltageSensor;
    private LowPassFilter velocityFilterUpper, velocityFilterLower;

    // Cache current tunables (to detect changes via FTCDashboard)
    private double _kP_UPPER, _kI_UPPER, _kD_UPPER, _kV_UPPER_NOMINAL, _kA_UPPER, _kS_UPPER_NOMINAL;
    private double _kP_LOWER, _kI_LOWER, _kD_LOWER, _kV_LOWER_NOMINAL, _kA_LOWER, _kS_LOWER_NOMINAL;
    private double _VEL_LP_ALPHA;
    private boolean _VOLTAGE_COMPENSATION_ENABLED;
    private double _NOMINAL_VOLTAGE;

    // Flywheel state control
    private boolean flywheelEnabled = false;

    // Voltage compensation cycle counter
    private int voltageCompCycleCounter = 0;

    // At-speed indicators (updated in periodic)
    private boolean atSpeedUpper = false;
    private boolean atSpeedLower = false;

    public static final Velauncher INSTANCE = new Velauncher();
    private Velauncher() { }

    private MotorEx upperMotor = new MotorEx("launch");
    private MotorEx lowerMotor = new MotorEx("launch2");

    @Override
    public void initialize(){
        // Get voltage sensor from hardware map
        voltageSensor = ActiveOpMode.hardwareMap().voltageSensor.iterator().next();

        // Apply motor direction configuration
        upperMotor.setDirection(-1);
        lowerMotor.setDirection(-1);

        buildControllers();
    }

    private void buildControllers() {
        // Calculate voltage compensation ratio
        double voltageRatio = 1.0;
        if (VOLTAGE_COMPENSATION_ENABLED && voltageSensor != null) {
            double currentVoltage = voltageSensor.getVoltage();
            voltageRatio = NOMINAL_VOLTAGE / currentVoltage;
        }

        // Apply voltage compensation to feedforward gains
        double kV_upper = kV_UPPER_NOMINAL * voltageRatio;
        double kV_lower = kV_LOWER_NOMINAL * voltageRatio;
        double kS_upper = kS_UPPER_NOMINAL * voltageRatio;
        double kS_lower = kS_LOWER_NOMINAL * voltageRatio;

        // Build upper motor controller
        ctrlUpper = ControlSystem.builder()
                .velPid(kP_UPPER, kI_UPPER, kD_UPPER)
                .basicFF(kV_upper, kA_UPPER, kS_upper)
                .build();

        // Build lower motor controller
        ctrlLower = ControlSystem.builder()
                .velPid(kP_LOWER, kI_LOWER, kD_LOWER)
                .basicFF(kV_lower, kA_LOWER, kS_lower)
                .build();

        // Build velocity low-pass filters if enabled
        if (VEL_LP_ALPHA >= 0.0 && VEL_LP_ALPHA <= 1.0) {
            velocityFilterUpper = new LowPassFilter(VEL_LP_ALPHA, upperMotor.getVelocity());
            velocityFilterLower = new LowPassFilter(VEL_LP_ALPHA, lowerMotor.getVelocity());
        } else {
            velocityFilterUpper = null;
            velocityFilterLower = null;
        }

        // Cache current values
        _kP_UPPER = kP_UPPER; _kI_UPPER = kI_UPPER; _kD_UPPER = kD_UPPER;
        _kV_UPPER_NOMINAL = kV_UPPER_NOMINAL; _kA_UPPER = kA_UPPER; _kS_UPPER_NOMINAL = kS_UPPER_NOMINAL;
        _kP_LOWER = kP_LOWER; _kI_LOWER = kI_LOWER; _kD_LOWER = kD_LOWER;
        _kV_LOWER_NOMINAL = kV_LOWER_NOMINAL; _kA_LOWER = kA_LOWER; _kS_LOWER_NOMINAL = kS_LOWER_NOMINAL;
        _VEL_LP_ALPHA = VEL_LP_ALPHA;
        _VOLTAGE_COMPENSATION_ENABLED = VOLTAGE_COMPENSATION_ENABLED;
        _NOMINAL_VOLTAGE = NOMINAL_VOLTAGE;

        // Reset cycle counter when controllers are rebuilt
        voltageCompCycleCounter = 0;
    }

    private boolean gainsChanged() {
        return _kP_UPPER != kP_UPPER || _kI_UPPER != kI_UPPER || _kD_UPPER != kD_UPPER
                || _kV_UPPER_NOMINAL != kV_UPPER_NOMINAL || _kA_UPPER != kA_UPPER || _kS_UPPER_NOMINAL != kS_UPPER_NOMINAL
                || _kP_LOWER != kP_LOWER || _kI_LOWER != kI_LOWER || _kD_LOWER != kD_LOWER
                || _kV_LOWER_NOMINAL != kV_LOWER_NOMINAL || _kA_LOWER != kA_LOWER || _kS_LOWER_NOMINAL != kS_LOWER_NOMINAL
                || _VEL_LP_ALPHA != VEL_LP_ALPHA
                || _VOLTAGE_COMPENSATION_ENABLED != VOLTAGE_COMPENSATION_ENABLED
                || _NOMINAL_VOLTAGE != NOMINAL_VOLTAGE;
    }

    public  Command velaunch = new LambdaCommand()
            .setStart(() -> {
                buildControllers();
                flywheelEnabled = true;
            })
            .setIsDone(() -> flywheelEnabled && atSpeedUpper && atSpeedLower);

    public  Command unvelaunch = new LambdaCommand() // lamb da command
            .setStart(() -> {
                flywheelEnabled = false;
            })
            .setIsDone(() -> !flywheelEnabled && !atSpeedUpper && !atSpeedLower);




    @Override
    public void periodic() {
        // Rebuild controllers if gains changed via FTCDashboard or voltage compensation cycle reached
        voltageCompCycleCounter++;
        boolean needsRebuild = gainsChanged() ||
                (VOLTAGE_COMPENSATION_ENABLED &&
                        VOLTAGE_COMP_REBUILD_CYCLES > 0 &&
                        voltageCompCycleCounter >= VOLTAGE_COMP_REBUILD_CYCLES);

        if (needsRebuild) {
            buildControllers();
        }

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

        // Get raw velocity measurements
        double rawVelUpper = Math.abs(upperMotor.getVelocity());
        double rawVelLower = Math.abs(lowerMotor.getVelocity());

        // Apply low-pass filter if enabled
        final double measTpsUpper = (velocityFilterUpper != null)
                ? velocityFilterUpper.filter(rawVelUpper)
                : rawVelUpper;
        final double measTpsLower = (velocityFilterLower != null)
                ? velocityFilterLower.filter(rawVelLower)
                : rawVelLower;
        final double measRpmUpper = tpsToRpm(measTpsUpper, TPR_UPPER);
        final double measRpmLower = tpsToRpm(measTpsLower, TPR_LOWER);

        final double errUpper = targetTpsUpper - measTpsUpper;
        final double errLower = targetTpsLower - measTpsLower;

        // Update at-speed indicators (used by commands)
        atSpeedUpper = Math.abs(errUpper) <= TOL_TPS_UPPER;
        atSpeedLower = Math.abs(errLower) <= TOL_TPS_LOWER;

        // Calculate voltage compensation metrics for telemetry
        double currentVoltage = (voltageSensor != null) ? voltageSensor.getVoltage() : 0.0;
        double voltageRatio = (VOLTAGE_COMPENSATION_ENABLED && currentVoltage > 0)
                ? NOMINAL_VOLTAGE / currentVoltage : 1.0;
        double kV_upper_compensated = kV_UPPER_NOMINAL * voltageRatio;
        double kV_lower_compensated = kV_LOWER_NOMINAL * voltageRatio;
        double kS_upper_compensated = kS_UPPER_NOMINAL * voltageRatio;
        double kS_lower_compensated = kS_LOWER_NOMINAL * voltageRatio;

        TelemetryPacket packet = new TelemetryPacket();

        // Main measurements
        packet.put("MEASURED UPPER RPM", measRpmUpper);
        packet.put("MEASURED LOWER RPM", measRpmLower);

        packet.put("---", "---");

        // Upper motor telemetry
        packet.put("Upper_target_tps", targetTpsUpper);
        packet.put("Upper_meas_tps", measTpsUpper);
        packet.put("Upper_target_rpm", TARGET_RPM_UPPER);
        packet.put("Upper_meas_rpm", measRpmUpper);
        packet.put("Upper_err_tps", errUpper);
        packet.put("Upper_power", powerUpper);
        packet.put("Upper_atSpeed", atSpeedUpper);

        // Lower motor telemetry
        packet.put("Lower_target_tps", targetTpsLower);
        packet.put("Lower_meas_tps", measTpsLower);
        packet.put("Lower_target_rpm", TARGET_RPM_LOWER);
        packet.put("Lower_meas_rpm", measRpmLower);
        packet.put("Lower_err_tps", errLower);
        packet.put("Lower_power", powerLower);
        packet.put("Lower_atSpeed", atSpeedLower);

        packet.put("---Voltage---", "---");

        // Voltage compensation telemetry
        packet.put("Battery_Voltage", currentVoltage);
        packet.put("Nominal_Voltage", NOMINAL_VOLTAGE);
        packet.put("Voltage_Ratio", voltageRatio);
        packet.put("Voltage_Comp_Enabled", VOLTAGE_COMPENSATION_ENABLED);
        packet.put("kV_Upper_Nominal", kV_UPPER_NOMINAL);
        packet.put("kV_Upper_Compensated", kV_upper_compensated);
        packet.put("kV_Lower_Nominal", kV_LOWER_NOMINAL);
        packet.put("kV_Lower_Compensated", kV_lower_compensated);
        packet.put("kS_Upper_Nominal", kS_UPPER_NOMINAL);
        packet.put("kS_Upper_Compensated", kS_upper_compensated);
        packet.put("kS_Lower_Nominal", kS_LOWER_NOMINAL);
        packet.put("kS_Lower_Compensated", kS_lower_compensated);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        Telemetry telemetry = ActiveOpMode.telemetry();
        telemetry.addData("Upper RPM", "%.0f / %.0f (err=%.0f) power=%.2f %s",
                measRpmUpper, TARGET_RPM_UPPER, tpsToRpm(errUpper, TPR_UPPER), powerUpper, atSpeedUpper ? "✓" : "✗");
        telemetry.addData("Lower RPM", "%.0f / %.0f (err=%.0f) power=%.2f %s",
                measRpmLower, TARGET_RPM_LOWER, tpsToRpm(errLower, TPR_LOWER), powerLower, atSpeedLower ? "✓" : "✗");
        telemetry.addData("Battery", "%.1fV (nominal: %.1fV) ratio: %.2fx",
                currentVoltage, NOMINAL_VOLTAGE, voltageRatio);
        telemetry.addData("Voltage Comp", VOLTAGE_COMPENSATION_ENABLED ? "ENABLED" : "DISABLED");
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