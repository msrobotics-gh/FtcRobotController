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

    public static double velocity1 = 65;

    public static double velocity2 = 65;
    private FeedbackElement pid = new PIDElement(FeedbackType.POSITION, coefficients);
    private ControlSystem controlSystem = ControlSystem.builder()
            .feedback(pid)
            .build();
    private ControlSystem controlSystem2 = ControlSystem.builder()
            .feedback(pid)
            .build();

    public Command velaunch = new RunToVelocity(controlSystem,velocity1);
    public Command velaunch2 = new RunToVelocity(controlSystem2,velocity2);

    public  Command kinelaunch = new RunToState(controlSystem,new KineticState(0.0, 65.0, 0.0), new KineticState(0.0, 2.0, 0.0));

    public  Command kinelaunch2 = new RunToState(controlSystem2,new KineticState(0.0, 65.0, 0.0), new KineticState(0.0, 2.0, 0.0));


    @Override
    public void periodic() {
        launch.setPower(controlSystem.calculate(launch.getState()));
        launch2.setPower(controlSystem.calculate(launch2.getState()));
    }
}


