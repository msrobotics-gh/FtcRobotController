package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.InstantCommand;
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

    public boolean lifted = false;

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

    private DistanceSensor sensorDistance;

    @Override
    public void initialize(){
        sensorDistance = ActiveOpMode.hardwareMap().get(DistanceSensor.class, "sensor_distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        lift_motor.zeroed();
        lift_motor2.zeroed();
    }

    // you can also cast this to a Rev2mDistanceSensor if you want to use added
    // methods associated with the Rev2mDistanceSensor class.
//    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
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

    public  Command liftHigh = new LambdaCommand()
            .setStart(() -> {
                buildControllers();
                lifted = true;
            })
            .setIsDone(() -> true);

//    public Command liftHigh = new InstantCommand(() -> {
//        controlSystem.setGoal(new KineticState(setPosition1));
//        controlSystem2.setGoal(new KineticState(setPosition2));
//        buildControllers();
//    });

    public  Command liftLow = new LambdaCommand()
            .setStart(() -> {
                buildControllers();
                lifted = false;
            })
            .setIsDone(() -> true);
//    public Command liftLow = new InstantCommand(() -> {
//        controlSystem.setGoal(new KineticState(0));
//        controlSystem2.setGoal(new KineticState(0));
//        buildControllers();
//    });

    @Override
    public void periodic() {


        liftPosition1 = lift_motor.getCurrentPosition();
        liftPosition2 = lift_motor2.getCurrentPosition();
        TelemetryPacket packet = new TelemetryPacket();

        if (lifted) {
            controlSystem.setGoal(new KineticState(setPosition1));
            controlSystem2.setGoal(new KineticState(setPosition2));
        } else {
            controlSystem.setGoal(new KineticState(0));
            controlSystem2.setGoal(new KineticState(0));

        }

        if (ActiveOpMode.opModeIsActive() && (!(sensorDistance.getDistance(DistanceUnit.INCH) > 12))){ //  && (setPosition1 == 0) && (setPosition2 == 0)
            lift_motor.setPower(controlSystem.calculate(lift_motor.getState()));
            lift_motor2.setPower(controlSystem2.calculate(lift_motor2.getState()));
        }
        else {
            lift_motor.setPower(0);
            lift_motor2.setPower(0);
        }

        packet.put("sensor_distance", sensorDistance.getDeviceName() );
        packet.put("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));

        // Main measurements
        packet.put("lift pos 1", liftPosition1);
        packet.put("lift pos 2", liftPosition2);

        packet.put("set pos 1",setPosition1);
        packet.put("set pos 2",setPosition2);


        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }
}
