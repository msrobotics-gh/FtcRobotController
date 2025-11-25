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
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "AUTONOMOUS BLUE")
public class DriveLaucnh extends NextFTCOpMode {
    public DriveLaucnh() {
        addComponents(
            BulkReadComponent.INSTANCE,
            new PedroComponent(Constants::createFollower),
            new SubsystemComponent(FlywheelGate.INSTANCE, Velauncher.INSTANCE)
        );
    }

    public void telem(String data) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("TELEMETRY", data);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private PathChain pathOne;

<<<<<<< HEAD
//    public SequentialGroup commandGroup;

    @Override
    public void onStartButtonPressed() {
        final Pose start = new Pose(0, 0, Math.toRadians(90));
        final Pose enddd = new Pose(0, Constants.AutonDistance, Math.toRadians(90));
        PedroComponent.follower().setStartingPose(start);
//        final PathChain pathOne;
        pathOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(start, enddd))
                .setLinearHeadingInterpolation(start.getHeading(), enddd.getHeading())
                .setVelocityConstraint(5)
                //.setConstantHeadingInterpolation(90.0)
                .build();

        Command pathGo = new FollowPath(pathOne);
//        Command turnGo = new TurnBy(Angle.fromDeg(Constants.blueDegrees));



        new SequentialGroup(
                Velauncher.INSTANCE.velaunch,
                new Delay(Constants.AutonDelay / 2),
                FlywheelGate.INSTANCE.open(),
                new Delay(Constants.AutonDelay / 2),


                Intake.INSTANCE.intake,
                Intake.INSTANCE.intakesecond,
                new Delay(Constants.AutonDelay),
                Intake.INSTANCE.intakeoff,
                Intake.INSTANCE.intakeoff2,

                new Delay(Constants.AutonDelay * 2),

                Intake.INSTANCE.intake,
                Intake.INSTANCE.intakesecond,
                new Delay(Constants.AutonDelay),
                Intake.INSTANCE.intakeoff,
                Intake.INSTANCE.intakeoff2,

                FlywheelGate.INSTANCE.close(),
                Velauncher.INSTANCE.unvelaunch,

//                new Delay(Constants.AutonDelay * 2),
//                turnGo,
                new Delay(Constants.AutonDelay * 1.5),
                pathGo

        ).schedule();
    }


//    @Override
//    public void onStartButtonPressed() {
////
////
////        int counter = 0;
////
////        Command tele = new LambdaCommand() // lamb da command
////            .setStart(() -> {
////                TelemetryPacket packet = new TelemetryPacket();
////                packet.put("Counter", counter);
////                FtcDashboard.getInstance().sendTelemetryPacket(packet);
////
////            })
////            .setIsDone(() -> true);
//
//
//        commandGroup.schedule();
//    }

    @Override
    public void onUpdate() {
//        TelemetryPacket packet = new TelemetryPacket();
//
//        // Main measurements
//        packet.put("robot x", PedroComponent.follower().getPose().getX());
//        packet.put("robot y", PedroComponent.follower().getPose().getY());
//
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}



////                Auto.INSTANCE.turnBlu, // to 60 for red; 122 for blue
////                new Delay(Constants.AutonDelay),
//            Velauncher.INSTANCE.velaunch,
//            new InstantCommand(()->{
//                TelemetryPacket packet = new TelemetryPacket();
//                packet.put("STATUS", "LAUNCH ON");
//                FtcDashboard.getInstance().sendTelemetryPacket(packet);
//            }),
//            new Delay(Constants.AutonDelay),
=======

    public Command firstRoutine() {
        Command pathGo = new FollowPath(pathOne);
        return new SequentialGroup(

                Velauncher.INSTANCE.velaunch,
                Intake.INSTANCE.intake,
                Intake.INSTANCE.intakesecond,
                new InstantCommand(() -> {
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("STATUS", "About to open gate");
                    packet.put("Servo position before", FlywheelGate.INSTANCE.gateServo.getPosition());
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                }),
                FlywheelGate.INSTANCE.open(),
                new InstantCommand(() -> {
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("STATUS", "Gate opened");
                    packet.put("Servo position after", FlywheelGate.INSTANCE.gateServo.getPosition());
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                })
                //pathGo
>>>>>>> 3075e38c8fa1cd71d3abdf0e308b3ed965f4f079

//
//                new ParallelGroup(
//                        Intake.INSTANCE.intake, // ball one
//                        Intake.INSTANCE.intakesecond
//                ),
//                new Delay(Constants.AutonDelay),
//                new ParallelGroup(
//                        Intake.INSTANCE.intakeoff, // ball one
//                        Intake.INSTANCE.intakeoff2
//                ),
//                new Delay(Constants.AutonDelay),
//
//                new ParallelGroup(
//                        Intake.INSTANCE.intake, // ball two
//                        Intake.INSTANCE.intakesecond
//                ),
//                new Delay(Constants.AutonDelay),
//                new ParallelGroup(
//                        Intake.INSTANCE.intakeoff, // ball two
//                        Intake.INSTANCE.intakeoff2
//                ),
//                new Delay(Constants.AutonDelay),
//
////                Intake.INSTANCE.intake, // ball three
////                Intake.INSTANCE.intakesecond,
////                new Delay(Constants.AutonDelay),
////                Intake.INSTANCE.intakeoff,
////                Intake.INSTANCE.intakeoff2,
////                new Delay(Constants.AutonDelay),
//
//                new ParallelGroup(
//                        FlywheelGate.INSTANCE.close(),
//                        Velauncher.INSTANCE.unvelaunch
//                ),
<<<<<<< HEAD
//                 pathGo
=======
//                 pathGo
        );
    }

    @Override
    public void onStartButtonPressed() {
        final Pose start = new Pose(0, 0, Math.toRadians(90));
        final Pose enddd = new Pose(0, 18, Math.toRadians(90));
        PedroComponent.follower().setStartingPose(start);
//        final PathChain pathOne;
        pathOne = PedroComponent.follower().pathBuilder()
            .addPath(new BezierLine(start, enddd))
            .setLinearHeadingInterpolation(start.getHeading(), enddd.getHeading())
            .setVelocityConstraint(5)
            //.setConstantHeadingInterpolation(90.0)
            .build();
        Command pathGo = new FollowPath(pathOne);

        int counter = 0;


        Command tele = new LambdaCommand() // lamb da command
            .setStart(() -> {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Counter", counter);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

            })
            .setIsDone(() -> true);

        firstRoutine().invoke();


//        pathGo.schedule();

        //Velauncher.INSTANCE.velaunch.schedule();
        //new Delay(0.5).schedule();
        //FlywheelGate.INSTANCE.open().schedule();
        //Intake.INSTANCE.intake.schedule();
        //Intake.INSTANCE.intakesecond.schedule();

    }

    @Override
    public void onUpdate() {
//        TelemetryPacket packet = new TelemetryPacket();
//
//        // Main measurements
//        packet.put("robot x", PedroComponent.follower().getPose().getX());
//        packet.put("robot y", PedroComponent.follower().getPose().getY());
//
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
>>>>>>> 3075e38c8fa1cd71d3abdf0e308b3ed965f4f079
