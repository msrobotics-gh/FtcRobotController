package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class Auto implements Subsystem {
    public static final Auto INSTANCE = new Auto();

    public static double height;
    private Auto() { }

    private MotorEx lift_motor = new MotorEx("lift_motor");

    private MotorEx lift_motor2 = new MotorEx("lift_motor2");

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();

    public Command toLow = new FollowPath();
//    public Command toHigh = new RunToPosition(controlSystem, -11300).requires(this);


    @Override
    public void periodic() {}
}
