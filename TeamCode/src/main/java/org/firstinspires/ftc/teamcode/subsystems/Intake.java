package org.firstinspires.ftc.teamcode.subsystems;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private CRServoEx intake1 = new CRServoEx("intake1");
    private CRServoEx intake2 = new CRServoEx("intake2");
    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();


    public Command intake = new SetPower(intake1,1);
    public Command intakesecond = new SetPower(intake2,1);

    public  Command intakeoff = new SetPower(intake1,0);

    public  Command intakeoff2 = new SetPower(intake2,0);

    @Override
    public void periodic() {
        intake1.setPower(controlSystem.calculate());
        intake2.setPower(controlSystem.calculate());
    }
}

