package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.rowanmcalpin.nextftc.pedro.TurnTo;


// if(pathTimer.getElapsedTimeSeconds() > 1) {}
// private Timer pathTimer, actionTimer, opmodeTimer;
// import com.pedropathing.util.Timer;
public class Auto implements Subsystem {
    public static final Auto INSTANCE = new Auto();
    private PathChain pathOne;

    private Auto() {
        final Pose pose1 = new Pose(72, 72, Math.toRadians(90));
        final Pose pose2 = new Pose(72, 96, Math.toRadians(90));
//        final PathChain pathOne;
        pathOne = follower().pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();
    }

    public Command follow = new FollowPath(pathOne, true);
    public Command turn = new TurnTo(Angle.fromDeg(180));


    @Override
    public void periodic() { }
}
