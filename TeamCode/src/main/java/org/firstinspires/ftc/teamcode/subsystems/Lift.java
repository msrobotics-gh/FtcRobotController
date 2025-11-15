package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class Lift implements Subsystem {
    public static final Lift INSTANCE = new Lift();
    public double liftPosition1;

    public double liftPosition2;

    public static double setPosition1 = -500;

    public static double setPosition2 = -500;
    public static double height;
    private Lift() { }
    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();
    private  ControlSystem controlSystem2 = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();
    private MotorEx lift_motor = new MotorEx("lift_motor");

    private MotorEx lift_motor2 = new MotorEx("lift_motor2");
    private void buildControllers(){
        controlSystem = ControlSystem.builder()
                .posPid(0.005, 0, 0)
                .elevatorFF(0)
                .build();
        controlSystem2 = ControlSystem.builder()
                .posPid(0.005, 0, 0)
                .elevatorFF(0)
                .build();


    }
    public Command toLow = new RunToPosition(controlSystem, 0).requires(this);
    public Command toHigh = new RunToPosition(controlSystem, -11300).requires(this);

    public Command toPos2 = new RunToPosition(controlSystem2, setPosition2).requires(this);
    public  Command run = new SetPower(lift_motor,-0.5);
    public  Command run2 = new SetPower(lift_motor2,-0.5);

    public  Command stop = new SetPower(lift_motor,0);
    public  Command stop2 = new SetPower(lift_motor2,0);

    public  Command toPos = new LambdaCommand()
            .setStart(() -> {
                buildControllers();
            })
            .setIsDone(() -> true);

    @Override
    public void periodic() {
        controlSystem.setGoal(new KineticState(setPosition1));
        controlSystem2.setGoal(new KineticState(setPosition2));
        liftPosition1 = lift_motor.getCurrentPosition();
        liftPosition2 = lift_motor2.getCurrentPosition();
        TelemetryPacket packet = new TelemetryPacket();

        // Main measurements
        packet.put("lift pos 1", liftPosition1);
        packet.put("lift pos 2", liftPosition2);

        packet.put("set pos 1",setPosition1);
        packet.put("set pos 2",setPosition2);


        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        lift_motor.setPower(controlSystem.calculate(lift_motor.getState()));
        lift_motor2.setPower(controlSystem2.calculate(lift_motor2.getState()));
    }
}
