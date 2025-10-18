package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class Lift implements Subsystem {
    public static final Lift INSTANCE = new Lift();

    public static double height = 50;
    private Lift() { }

    private MotorEx lift_motor = new MotorEx("left_lift_motor");

    private MotorEx lift_motor2 = new MotorEx("right_lift_motor");

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();

    public Command toLow = new RunToPosition(controlSystem, 0).requires(this);
    public Command toHigh = new RunToPosition(controlSystem, height).requires(this);

    public Command toHigh(double targetHeight){
        ActiveOpMode.telemetry().addLine("Running to height");
        return new RunToPosition(controlSystem, targetHeight);
    }

    @Override
    public void periodic() {
        ActiveOpMode.telemetry().addData("Current height: ", lift_motor.getCurrentPosition());
        lift_motor.setPower(controlSystem.calculate(lift_motor.getState()));
        lift_motor2.setPower(controlSystem.calculate(lift_motor2.getState()));
    }
}
