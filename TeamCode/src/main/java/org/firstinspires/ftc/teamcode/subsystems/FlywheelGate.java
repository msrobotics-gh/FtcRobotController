package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;


public class FlywheelGate implements Subsystem {
    public static final FlywheelGate INSTANCE = new FlywheelGate();
    private FlywheelGate() { }

    public Servo gateServo;
    public ServoEx gateServoEx;

    public String name = "gate_servo";

    @Override
    public void initialize() {
        gateServo = ActiveOpMode.hardwareMap().get(Servo.class, name);
        gateServoEx = new ServoEx(gateServo);
    }
    
    public Command open() {
        return new SequentialGroup(
            new SetPosition(
                    gateServoEx, 0.0
            ),

        new LambdaCommand().setStart(
                () -> {
                    ActiveOpMode.telemetry().addLine("Opening");
                    ActiveOpMode.telemetry().update();
                }
        )
        );
    }

    public Command close() {
        return new SequentialGroup(
                new SetPosition(
                        gateServoEx, 0.3
                ),

                new LambdaCommand().setStart(
                        () -> {
                            ActiveOpMode.telemetry().addLine("Closing");
                            ActiveOpMode.telemetry().update();
                        }
                )
        );
    }
}
