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
import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Velauncher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "simple auton launch")
public class DriveLaucnh extends NextFTCOpMode {
    public DriveLaucnh() {
        addComponents(
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(FlywheelGate.INSTANCE),
                new SubsystemComponent(Velauncher.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE)
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


         new SequentialGroup(
//                Auto.INSTANCE.turnBlu, // to 60 for red; 122 for blue
//                new Delay(Constants.AutonDelay),
                Velauncher.INSTANCE.velaunch,
                new Delay(Constants.AutonDelay),
                FlywheelGate.INSTANCE.open(),

                new ParallelGroup(
                        Intake.INSTANCE.intake, // ball one
                        Intake.INSTANCE.intakesecond
                ),
                new Delay(Constants.AutonDelay),
                new ParallelGroup(
                        Intake.INSTANCE.intakeoff, // ball one
                        Intake.INSTANCE.intakeoff2
                ),
                new Delay(Constants.AutonDelay),

                new ParallelGroup(
                        Intake.INSTANCE.intake, // ball two
                        Intake.INSTANCE.intakesecond
                ),
                new Delay(Constants.AutonDelay),
                new ParallelGroup(
                        Intake.INSTANCE.intakeoff, // ball two
                        Intake.INSTANCE.intakeoff2
                ),
                new Delay(Constants.AutonDelay),

//                Intake.INSTANCE.intake, // ball three
//                Intake.INSTANCE.intakesecond,
//                new Delay(Constants.AutonDelay),
//                Intake.INSTANCE.intakeoff,
//                Intake.INSTANCE.intakeoff2,
//                new Delay(Constants.AutonDelay),

                new ParallelGroup(
                        FlywheelGate.INSTANCE.close(),
                        Velauncher.INSTANCE.unvelaunch
                ),
                 pathGo
        ).schedule();


//        pathGo.schedule();

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