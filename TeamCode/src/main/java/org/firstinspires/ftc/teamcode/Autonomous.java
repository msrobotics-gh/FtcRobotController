package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.ConstantsFused;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Velauncher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends NextFTCOpMode {
    public Autonomous() {
        addComponents(
            BulkReadComponent.INSTANCE,
            new PedroComponent(Constants::createFollower),
            new SubsystemComponent(FlywheelGate.INSTANCE, Velauncher.INSTANCE, Intake.INSTANCE)
        );
    }

    private PathChain pathOne;

    public int commandNumber = 0;


    public Command auto(PathChain pathOne) {
        Command pathGo = new FollowPath(pathOne);
        Command incr = new InstantCommand(()->commandNumber++);

        return new SequentialGroup(
                // reset everything
                Velauncher.INSTANCE.unvelaunch, incr,
                Intake.INSTANCE.intakeoff, incr,
                Intake.INSTANCE.intakeoff2, incr,
                FlywheelGate.INSTANCE.close(), incr,

                new Delay(Constants.AutonDelay), incr,

                // -- autonomous start --

                Velauncher.INSTANCE.velaunch, incr,
                Intake.INSTANCE.intake, incr,
                Intake.INSTANCE.intakesecond, incr,
                FlywheelGate.INSTANCE.open(), incr,

                new Delay(Constants.AutonDelay * 12), incr,

                pathGo, incr
        );
    }

    @Override
    public void onStartButtonPressed() {
        final Pose start = new Pose(72, 72, Math.toRadians(90));
        final Pose enddd = new Pose(72, 72 + (Constants.AutonDistance * 4), Math.toRadians(90)); // autondistance is -6

        PedroComponent.follower().setStartingPose(start);
        PedroComponent.follower().setMaxPower(0.5);

        pathOne = PedroComponent.follower().pathBuilder()
            .addPath(new BezierLine(start, enddd))
            .setLinearHeadingInterpolation(start.getHeading(), enddd.getHeading())
//            .setVelocityConstraint(5)
            .build();

        auto(pathOne).schedule();
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