package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;

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
    public static final Velauncher INSTANCE = new Velauncher();
    private Velauncher() { }

    private MotorEx launch = new MotorEx("launch").reversed();
    private MotorEx launch2 = new MotorEx("launch2").reversed();

    public static double velocity1 = rpmToTps(100,28);

    public static double unvelocity1 = rpmToTps(0,28);

    public static double unvelocity2 = rpmToTps(0,28);


    public static double velocity2 = rpmToTps(100,28);

     double targetTpsUpper = rpmToTps(TARGET_RPM_UPPER, TPR_UPPER);
   double targetTpsLower = rpmToTps(TARGET_RPM_LOWER, TPR_LOWER);

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

    public  Command velaunch = new LambdaCommand()
            .setStart(() -> {
                ctrlUpper.setGoal(new KineticState(0.0, targetTpsUpper, 0.0));
            })
            .setIsDone(() -> true);
    public  Command velaunch2 = new LambdaCommand()
            .setStart(() -> {
                ctrlLower.setGoal(new KineticState(0.0, targetTpsLower, 0.0));
            })
            .setIsDone(() -> true);


    public  Command unvelaunch = new LambdaCommand()
            .setStart(() -> {
                ctrlUpper.setGoal(new KineticState(0.0, 0, 0.0));
            })
            .setIsDone(() -> true);

    public  Command unvelaunch2 = new LambdaCommand()
            .setStart(() -> {
                ctrlLower.setGoal(new KineticState(0.0, 0, 0.0));
            })
            .setIsDone(() -> true);



    @Override
    public void periodic() {
        launch.setPower(ctrlLower.calculate(launch.getState()));
        launch2.setPower(ctrlUpper.calculate(launch2.getState()));
         targetTpsUpper = rpmToTps(TARGET_RPM_UPPER, TPR_UPPER);
         targetTpsLower = rpmToTps(TARGET_RPM_LOWER, TPR_LOWER);

         
    }
    private static double rpmToTps(double rpm, double tpr) {
        return rpm * tpr / 60.0;
    }

    private static double tpsToRpm(double tps, double tpr) {
        return (tps * 60.0) / tpr;
    }
}


