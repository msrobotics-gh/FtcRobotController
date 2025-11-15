package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Const;
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

//

public class Auto implements Subsystem {
    public static final Auto INSTANCE = new Auto();
    private PathChain pathOne = new PathChain();

    private Auto() {
        final Pose start = new Pose(72, 72, Math.toRadians(90));
        final Pose enddd = new Pose(72, 72 + Constants.AutonDistance, Math.toRadians(90));

        follower().setStartingPose(start);
//        final PathChain pathOne;
        pathOne = follower().pathBuilder()
                .addPath(new BezierLine(start, enddd))
//                .setLinearHeadingInterpolation(start.getHeading(), pose2.getHeading())
                .setConstantHeadingInterpolation(90.0)
                .build();

    }

    public Command follow = new FollowPath(pathOne, true);
    public Command turnBlu = new TurnTo(Angle.fromDeg(Constants.blueDegrees));
    public Command turnRed = new TurnTo(Angle.fromDeg(Constants.redDegrees));


    @Override
    public void periodic() { }
}
