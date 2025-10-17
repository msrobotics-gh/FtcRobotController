package org.firstinspires.ftc.teamcode.subsystems;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.Powerable;
import dev.nextftc.hardware.powerable.SetPower;

public class Launcher implements Subsystem {
    public static final Launcher INSTANCE = new Launcher();
    private Launcher() { }

    private MotorEx launch = new MotorEx("launch");
    private MotorEx launch2 = new MotorEx("launch2");

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();

    public Command move = new SetPower(launch,1).requires(this);
    public Command move2 = new SetPower(launch2,1).requires(this);

    @Override
    public void periodic() {
        launch.setPower(controlSystem.calculate(launch.getState()));
        launch2.setPower(controlSystem.calculate(launch2.getState()));
    }
}

