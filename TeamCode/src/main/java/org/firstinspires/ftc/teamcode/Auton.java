package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Velauncher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import kotlinx.coroutines.selects.SelectUnbiasedKt;

@Autonomous(name = "Autonomous")
public class Auton extends NextFTCOpMode {
    public Auton() {
        addComponents(
            BulkReadComponent.INSTANCE,
            new PedroComponent(Constants::createFollower),
            new SubsystemComponent(FlywheelGate.INSTANCE, Velauncher.INSTANCE, Intake.INSTANCE)
        );
    }

    private PathChain forward, intakeP;

    public int commandNumber = 0;


    public Command auto(PathChain pathOne, PathChain pathTwo) {
        Command pathGo = new FollowPath(pathOne);
        Command pathGo2 = new FollowPath(pathTwo);
        Command incr = new InstantCommand(()->commandNumber++);
        Command reset = new SequentialGroup(
                Velauncher.INSTANCE.unvelaunch, incr,
                new Delay(Constants.AutonDelay / 2), incr,
                Intake.INSTANCE.intakeoff, incr,
                Intake.INSTANCE.intakeoff2, incr,
                FlywheelGate.INSTANCE.close(), incr
        );
        Command start = new SequentialGroup(
                Velauncher.INSTANCE.velaunch, incr,
                new Delay(Constants.AutonDelay / 2), incr,
                Intake.INSTANCE.intake, incr,
                Intake.INSTANCE.intakesecond, incr,
                FlywheelGate.INSTANCE.open(), incr
        );

        return new SequentialGroup(
                // reset everything

                reset, incr,

                new Delay(Constants.AutonDelay), incr,

                // -- autonomous start --

                start, incr,

                new Delay(20), incr,

                reset, incr,

                pathGo, incr
//                new Delay(Constants.AutonDelay * 12), incr,
//                pathGo2, incr,
//
//
//                start, incr


        );
    }

    @Override
    public void onStartButtonPressed() {
        final Pose start = new Pose(75, 10, Math.toRadians(119));
        final Pose enddd = new Pose(75, 10 + (Constants.AutonDistance), Math.toRadians(90));
        final Pose intak = new Pose(10, 10, Math.toRadians(0));
        // AutonDistance is one tile, -6

        PedroComponent.follower().setStartingPose(start);
        PedroComponent.follower().setMaxPower(0.2);

        forward = PedroComponent.follower().pathBuilder()
            .addPath(new BezierLine(start, enddd))
            .setLinearHeadingInterpolation(start.getHeading(), enddd.getHeading())
            .build();

        intakeP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(enddd, intak))
                .setLinearHeadingInterpolation(enddd.getHeading(), intak.getHeading())
                .build();

        new SequentialGroup(
            auto(forward, intakeP)
        ).schedule();
    }

    @Override
    public void onUpdate() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Command number", commandNumber);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }


    @Override
    public void onStop() {
        new SequentialGroup(
                Velauncher.INSTANCE.unvelaunch,
                Intake.INSTANCE.intakeoff,
                Intake.INSTANCE.intakeoff2,
                FlywheelGate.INSTANCE.close()
        ).schedule();
    }
}