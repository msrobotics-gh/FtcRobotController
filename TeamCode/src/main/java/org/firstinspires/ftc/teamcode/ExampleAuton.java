package org.firstinspires.ftc.teamcode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.Auto;
//import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Velauncher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;



@Autonomous(name = "EXAMPLE AUTON")
public class ExampleAuton extends NextFTCOpMode {
    public ExampleAuton() {
        addComponents(
                new SubsystemComponent(FlywheelGate.INSTANCE, Velauncher.INSTANCE, Intake.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                FlywheelGate.INSTANCE.open(),
                new ParallelGroup(
                        Intake.INSTANCE.intake,
                        Intake.INSTANCE.intakesecond
                ),
                new Delay(Constants.AutonDelay * 7),
                new ParallelGroup(
                        Intake.INSTANCE.intakeoff,
                        Intake.INSTANCE.intakeoff2
                ),
                FlywheelGate.INSTANCE.close(),
                Velauncher.INSTANCE.velaunch
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }
}