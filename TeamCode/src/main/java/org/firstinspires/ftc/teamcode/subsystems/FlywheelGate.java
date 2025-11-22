package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import kotlin.time.Instant;


public class FlywheelGate implements Subsystem {
    public static final FlywheelGate INSTANCE = new FlywheelGate();

    private FlywheelGate() {
    }

    public Servo gateServo;
    public ServoEx gateServoEx;

    public String name = "gate_servo";

    @Override
    public void initialize() {
        gateServo = ActiveOpMode.hardwareMap().get(Servo.class, name);
        gateServoEx = new ServoEx(gateServo);
        gateServoEx.setPosition(0.3);
    }


    public Command open() {
        return new SetPosition(gateServoEx, // SERVO TO MOVE
                0.0); // IMPLEMENTED SUBSYSTEM
    }

    public Command close() {
        return new SetPosition(gateServoEx, // SERVO TO MOVE
                0.3); // IMPLEMENTED SUBSYSTEM
    }


//    public Command open = new SetPosition(gateServoEx, 0.0);

    public Command openv2 = new LambdaCommand()
            .setStart(()-> { new SetPosition(gateServoEx, 0.0).start(); })
            .setIsDone(() -> (gateServoEx.getPosition() < 0.1));

//    public Command close = new SetPosition(gateServoEx, 0.3);

    public Command closev2 = new LambdaCommand()
            .setStart(()-> { new SetPosition(gateServoEx, 0.3).start(); })
            .setIsDone(() -> (gateServoEx.getPosition() > 0.29));

    // SetPosition inherits class Command
}
