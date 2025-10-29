package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.net.PortUnreachableException;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.FeedbackElement;
import dev.nextftc.control.feedback.FeedbackType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDElement;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToState;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.delegates.Velocity;
import dev.nextftc.hardware.driving.HolonomicDrivePowers;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.Powerable;
import dev.nextftc.hardware.powerable.SetPower;

/**
 * Flywheel velocity tuning OpMode using FTCDashboard.
 * - Independent velocity control for two flywheel motors (upper and lower)
 * - Live-tunable PIDF (velPid + feedforward kV/kA/kS)
 * - Displays target/measured velocity, error, and power on Dashboard
 */
@Config
@TeleOp(name = "FlywheelVelTuner")
public class FlywheelVelTuner extends NextFTCOpMode {

    public FlywheelVelTuner() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE)
        );
    }
    // === Hardware names (change to match your configuration) ===
    public static String UPPER_MOTOR_NAME = "launch";
    public static String LOWER_MOTOR_NAME = "launch2";

    // === Encoder / unit config (per motor) ===
    public static double TPR_UPPER = 28; // ticks per output revolution (upper motor)
    public static double TPR_LOWER = 28; // ticks per output revolution (lower motor)

    // === Target speeds (RPM) ===
    public static double TARGET_RPM_UPPER = 500;
    public static double TARGET_RPM_LOWER = 500;

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
    public static double kV_UPPER = 0.00152174;
    public static double kA_UPPER = 0.0;
    public static double kS_UPPER = 0.02;

    // === NextFTC Control gains — Lower motor ===
    public static double kP_LOWER = 0.00002;
    public static double kI_LOWER = 0.0;
    public static double kD_LOWER = 0.0;
    public static double kV_LOWER = 0.00152174;
    public static double kA_LOWER = 0.0;
    public static double kS_LOWER = 0.02;

    // === Optional velocity low-pass ===
    public static double VEL_LP_ALPHA = -1.0; // e.g., 0.2 for filtering

    // === Directions ===
    public static boolean REVERSE_UPPER = false;
    public static boolean REVERSE_LOWER = true;

    private MotorEx upperMotor, lowerMotor;
    private ControlSystem ctrlUpper, ctrlLower;

    // Cache current tunables
    private double _kP_UPPER, _kI_UPPER, _kD_UPPER, _kV_UPPER, _kA_UPPER, _kS_UPPER;
    private double _kP_LOWER, _kI_LOWER, _kD_LOWER, _kV_LOWER, _kA_LOWER, _kS_LOWER;
    private double _VEL_LP_ALPHA;

    @Override
    public void onInit() {
        upperMotor = new MotorEx(UPPER_MOTOR_NAME);
        lowerMotor = new MotorEx(LOWER_MOTOR_NAME);

        upperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //upperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lowerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //upperMotor.setDirection(REVERSE_UPPER ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        //lowerMotor.setDirection(REVERSE_LOWER ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        buildControllers();

        telemetry.addLine("FlywheelVelTuner ready. Adjust params in FTCDashboard.");
        telemetry.update();
    }

  

    @Override
    public void onUpdate() {
        if (gainsChanged()) buildControllers();

        final double targetTpsUpper = rpmToTps(TARGET_RPM_UPPER, TPR_UPPER);
        final double targetTpsLower = rpmToTps(TARGET_RPM_LOWER, TPR_LOWER);

        double powerUpper, powerLower;

        if (CLOSED_LOOP_ENABLED) {
            ctrlUpper.setGoal(new KineticState(0.0, targetTpsUpper, 0.0));
            ctrlLower.setGoal(new KineticState(0.0, targetTpsLower, 0.0));

            powerUpper = ctrlUpper.calculate(upperMotor.getState());
            powerLower = ctrlLower.calculate(lowerMotor.getState());
        } else {
            powerUpper = OPEN_LOOP_POWER_UPPER;
            powerLower = OPEN_LOOP_POWER_LOWER;
        }

        if (ENABLED_UPPER) upperMotor.setPower(powerUpper); else upperMotor.setPower(0);
        if (ENABLED_LOWER) lowerMotor.setPower(powerLower); else lowerMotor.setPower(0);

        final double measTpsUpper = upperMotor.getVelocity();
        final double measTpsLower = lowerMotor.getVelocity();
        final double measRpmUpper = tpsToRpm(measTpsUpper, TPR_UPPER);
        final double measRpmLower = tpsToRpm(measTpsLower, TPR_LOWER);

        final double errUpper = targetTpsUpper - measTpsUpper;
        final double errLower = targetTpsLower - measTpsLower;

        final boolean atSpeedUpper = Math.abs(errUpper) <= TOL_TPS_UPPER;
        final boolean atSpeedLower = Math.abs(errLower) <= TOL_TPS_LOWER;

        TelemetryPacket packet = new TelemetryPacket();
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

        telemetry.addData("Upper RPM", "%.0f / %.0f (err=%.0f) power=%.2f %s",
                measRpmUpper, TARGET_RPM_UPPER, tpsToRpm(errUpper, TPR_UPPER), powerUpper, atSpeedUpper ? "✓" : "✗");
        telemetry.addData("Lower RPM", "%.0f / %.0f (err=%.0f) power=%.2f %s",
                measRpmLower, TARGET_RPM_LOWER, tpsToRpm(errLower, TPR_LOWER), powerLower, atSpeedLower ? "✓" : "✗");
        telemetry.addData("Closed Loop", CLOSED_LOOP_ENABLED);
        telemetry.update();
    }

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

    private static double rpmToTps(double rpm, double tpr) {
        return rpm * tpr / 60.0;
    }

    private static double tpsToRpm(double tps, double tpr) {
        return (tps * 60.0) / tpr;
    }
}