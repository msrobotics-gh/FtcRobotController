package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Velauncher;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "simple auton launch")
public class TestAuton extends NextFTCOpMode {
    public TestAuton() {
        addComponents(
                BulkReadComponent.INSTANCE
        );
    }


    @Override
    public void onStartButtonPressed() {
        Velauncher.INSTANCE.velaunch.schedule();
        new Delay(0.5).schedule();
        FlywheelGate.INSTANCE.open().schedule();
        Intake.INSTANCE.intake.schedule();
        Intake.INSTANCE.intakesecond.schedule();

    }
}