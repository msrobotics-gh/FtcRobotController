package org.firstinspires.ftc.teamcode.subsystems;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.driving.HolonomicDrivePowers;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.Powerable;
import dev.nextftc.hardware.powerable.SetPower;

public class Launcher implements Subsystem {
    public static final Launcher INSTANCE = new Launcher();
    private Launcher() { }

    private MotorEx launch = new MotorEx("launch").reversed();
    private MotorEx launch2 = new MotorEx("launch2").reversed();

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();


    public Command move = new SetPower(launch,.7);
    public Command move2 = new SetPower(launch2,.7);

    public Command dontmove = new SetPower(launch,0);
    public Command dontmove2 = new SetPower(launch2,0);

    @Override
    public void periodic() {
        launch.setPower(controlSystem.calculate(launch.getState()));
        launch2.setPower(controlSystem.calculate(launch2.getState()));
    }
}

