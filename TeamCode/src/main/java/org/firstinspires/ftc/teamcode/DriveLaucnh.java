package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Velauncher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
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

    public int commandNumber = 0;


    public Command firstRoutine() {
        Command pathGo = new FollowPath(pathOne);
        Command incr = new InstantCommand(()->{
            commandNumber++;
        });

        return new SequentialGroup(

                Velauncher.INSTANCE.velaunch, incr,
                Intake.INSTANCE.intake, incr,
                Intake.INSTANCE.intakesecond, incr,
                new InstantCommand(() -> {
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("STATUS", "About to open gate");
                    packet.put("Servo position before", FlywheelGate.INSTANCE.gateServo.getPosition());
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                }), incr,
                FlywheelGate.INSTANCE.open(), incr,
                new InstantCommand(() -> {
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("STATUS", "Gate opened");
                    packet.put("Servo position after", FlywheelGate.INSTANCE.gateServo.getPosition());
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                }), incr
                //pathGo

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
//                 pathGo
        );
    }

    @Override
    public void onStartButtonPressed() {
        final Pose start = new Pose(0, 0, Math.toRadians(90));
        final Pose enddd = new Pose(0, Constants.AutonDistance, Math.toRadians(90)); // autondistance is 18
        PedroComponent.follower().setStartingPose(start);
//        final PathChain pathOne;
        pathOne = PedroComponent.follower().pathBuilder()
            .addPath(new BezierLine(start, enddd))
            .setLinearHeadingInterpolation(start.getHeading(), enddd.getHeading())
            .setVelocityConstraint(5)
            //.setConstantHeadingInterpolation(90.0)
            .build();


//        int counter = 0;
//
//
//        Command tele = new LambdaCommand() // lamb da command
//            .setStart(() -> {
//                TelemetryPacket packet = new TelemetryPacket();
//                packet.put("Counter", counter);
//                FtcDashboard.getInstance().sendTelemetryPacket(packet);
//
//            })
//            .setIsDone(() -> true);

        firstRoutine().schedule();


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
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Command number", commandNumber);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}