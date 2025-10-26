package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "bartholomew is back", group = "Autonomous")
@Config // 7.44688232853678 // 0.1357132846203596 // 0.09280719038382527
public class ta extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

//    public int poseOneX = 72;
//    public int poseOneY = 96;
//    public int poseTwoX = 96;
//    public int poseTwoY = 96;
//
//    public int poseOneT = 90;
//    public int poseTwoT = 90;

    public int waitTime = 1;

    // POSES

    private final Pose pose1 = new Pose(72, 72, Math.toRadians(90));
    private final Pose pose2 = new Pose(72, 96, Math.toRadians(90)); // forward 48
    private final Pose pose3 = new Pose(96, 96, Math.toRadians(90)); // left 48
    private final Pose pose4 = new Pose(96,24, Math.toRadians(90));
    private final Pose pose5 = new Pose(24,24, Math.toRadians(90));
    private final Pose pose6 = new Pose(24,96, Math.toRadians(90));

    //    private Path;
    private PathChain pathOne, pathTwo, pathThr, pathFou, pathFiv, pathSix, pathSev;
//    private PathChain pathTwo;


    public void buildPaths() {

        // completePath = follower.pathBuilder()
        //         .addPath(new BezierLine(pose1, pose2))
        //         .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
        //         .addPath(new BezierLine(pose2, pose3))
        //         .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
        //         .addPath(new BezierLine(pose1, pose2))
        //         .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
        //         .addPath(new BezierLine(pose2, pose3))
        //         .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
        //         .addPath(new BezierLine(pose3, pose4))
        //         .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
        //         .addPath(new BezierLine(pose4, pose5))
        //         .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
        //         .addPath(new BezierLine(pose5, pose6))
        //         .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
        //         .addPath(new BezierLine(pose6, pose2))
        //         .setLinearHeadingInterpolation(pose6.getHeading(), pose2.getHeading())
        //         .addPath(new BezierLine(pose2, pose1))
        //         .setLinearHeadingInterpolation(pose2.getHeading(), pose1.getHeading())
        //         .build();

        pathOne = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        pathTwo = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();

        pathThr = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();

        pathFou = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose5))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
                .build();

        pathFiv = follower.pathBuilder()
                .addPath(new BezierLine(pose5, pose6))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
                .build();

        pathSix = follower.pathBuilder()
                .addPath(new BezierLine(pose6, pose2))
                .setLinearHeadingInterpolation(pose6.getHeading(), pose2.getHeading())
                .build();

        pathSev = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose1))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose1.getHeading())
                .build();

        // pathThree = follower.pathBuilder()
        //         .addPath(new BezierLine(pose1, pose2))
        //         .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
        //         .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathOne);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(pathTwo);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pathThr);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pathFou);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pathFiv);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pathSix);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(pathSev, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.holdPoint(follower.getPose());
                    setPathState(-1);
                }
                break;


//- Follower State: "if(!follower.isBusy()) {}"
//- Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//- Robot Position: "if(follower.getPose().getX() > 36) {}"


        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    // run repeatedly after clicking Start
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        dashboardTelemetry.addData("path state", pathState);
        dashboardTelemetry.addData("x", follower.getPose().getX());
        dashboardTelemetry.addData("y", follower.getPose().getY());
        dashboardTelemetry.addData("heading", follower.getPose().getHeading());
        dashboardTelemetry.update();
    }

    // called once
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(pose1);

    }

    // called continuously while waiting for Start
    @Override
    public void init_loop() {}

    // called once at start of opmode
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    // dont use; everything auto disable
    @Override
    public void stop() {}

}