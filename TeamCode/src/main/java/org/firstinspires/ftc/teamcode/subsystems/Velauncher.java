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
    public static final Velauncher INSTANCE = new Velauncher();
    private Velauncher() { }

    private MotorEx launch = new MotorEx("launch").reversed();
    private MotorEx launch2 = new MotorEx("launch2").reversed();
    public static PIDCoefficients coefficients = new  PIDCoefficients(0.1,0,0);

    public static double velocity1 = rpmToTps(100,28);

    public static double unvelocity1 = rpmToTps(0,28);

    public static double unvelocity2 = rpmToTps(0,28);


    public static double velocity2 = rpmToTps(100,28);

    public static double TPR_UPPER = 28;
    public static double TPR_LOWER = 28;

    public static double TARGET_RPM_UPPER = 1000;
    public static double TARGET_RPM_LOWER = 1000;
    //private FeedbackElement pid = new PIDElement(FeedbackType.VELOCITY, coefficients);
    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.01, 0, 0)
            .build();
    private ControlSystem controlSystem2 = ControlSystem.builder()
            .velPid(0.01, 0, 0)
            .build();

    public Command velaunch = new RunToVelocity(controlSystem,velocity1);
    public Command velaunch2 = new RunToVelocity(controlSystem2,velocity2);

    public Command unvelaunch = new RunToVelocity(controlSystem,unvelocity1);

    public  Command unvelaunch2 = new RunToVelocity(controlSystem2,unvelocity2);



    @Override
    public void periodic() {
        launch.setPower(controlSystem.calculate(launch.getState()));
        launch2.setPower(controlSystem2.calculate(launch2.getState()));
    }
    private static double rpmToTps(double rpm, double tpr) {
        return rpm * tpr / 60.0;
    }

    private static double tpsToRpm(double tps, double tpr) {
        return (tps * 60.0) / tpr;
    }
}


