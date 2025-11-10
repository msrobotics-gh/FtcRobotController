package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Velauncher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
// dont need lift


import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "AUTON RED ALLIANCE", group = "Autonomous")
@Config
public class DriveLaunchRed extends NextFTCOpMode {
    public DriveLaunchRed() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(FlywheelGate.INSTANCE),
                new SubsystemComponent(Velauncher.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(Auto.INSTANCE)

        );
    }

    public void tele(String data) {
        telemetry.addData("> ",data);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        telemetry.addLine("RED ALLIANCE AUTONOMOUS");
        telemetry.update();
//        Auto.INSTANCE.turnRed.schedule(); // to 60 for red; 122 for blue
//        new Delay(0.5).schedule(); // wait for turn
//        Velauncher.INSTANCE.velaunch.schedule();
//        new Delay(0.5).schedule();
//        FlywheelGate.INSTANCE.open().schedule();
//        for (int i=0; i<3; i++){ // thrice - each ball
//            Intake.INSTANCE.intake.schedule();
//            Intake.INSTANCE.intakesecond.schedule();
//            new Delay(0.5).schedule();
//            Intake.INSTANCE.intakeoff.schedule();
//            Intake.INSTANCE.intakeoff2.schedule();
//            new Delay(0.5).schedule();
//        }
//        FlywheelGate.INSTANCE.close().schedule();
//        Velauncher.INSTANCE.unvelaunch.schedule();
//        Auto.INSTANCE.follow.schedule();


        new SequentialGroup(
                Auto.INSTANCE.turnRed, // to 60 for red; 122 for blue
                new Delay(0.5),
                Velauncher.INSTANCE.velaunch,
                new Delay(0.5),
                FlywheelGate.INSTANCE.open(),

                Intake.INSTANCE.intake, // ball one
                Intake.INSTANCE.intakesecond,
                new Delay(0.5),
                Intake.INSTANCE.intakeoff,
                Intake.INSTANCE.intakeoff2,
                new Delay(0.5),

                Intake.INSTANCE.intake, // ball two
                Intake.INSTANCE.intakesecond,
                new Delay(0.5),
                Intake.INSTANCE.intakeoff,
                Intake.INSTANCE.intakeoff2,
                new Delay(0.5),

                Intake.INSTANCE.intake, // ball three
                Intake.INSTANCE.intakesecond,
                new Delay(0.5),
                Intake.INSTANCE.intakeoff,
                Intake.INSTANCE.intakeoff2,
                new Delay(0.5),

                FlywheelGate.INSTANCE.close(),
                Velauncher.INSTANCE.unvelaunch,
                Auto.INSTANCE.follow


        ).schedule();


    }
}
