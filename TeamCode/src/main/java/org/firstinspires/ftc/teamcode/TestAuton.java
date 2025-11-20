package org.firstinspires.ftc.teamcode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Velauncher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "simple auton launch")
public class TestAuton extends NextFTCOpMode {
    public TestAuton() {
        addComponents(
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    private PathChain pathOne;

    @Override
    public void onStartButtonPressed() {
        final Pose start = new Pose(0, 0, Math.toRadians(90));
        final Pose enddd = new Pose(0, 6, Math.toRadians(90));
        PedroComponent.follower().setStartingPose(start);
//        final PathChain pathOne;
        pathOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(start, enddd))
                .setLinearHeadingInterpolation(start.getHeading(), enddd.getHeading())
                .setVelocityConstraint(5)
                //.setConstantHeadingInterpolation(90.0)
                .build();
        Command pathGo = new FollowPath(pathOne);
        pathGo.schedule();

        //Velauncher.INSTANCE.velaunch.schedule();
        //new Delay(0.5).schedule();
        //FlywheelGate.INSTANCE.open().schedule();
        //Intake.INSTANCE.intake.schedule();
        //Intake.INSTANCE.intakesecond.schedule();

    }

    @Override
    public void onUpdate() {
        TelemetryPacket packet = new TelemetryPacket();

        // Main measurements
        packet.put("robot x", PedroComponent.follower().getPose().getX());
        packet.put("robot y", PedroComponent.follower().getPose().getY());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}