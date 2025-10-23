package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "drive in line", group = "PP")
public class tanext extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // POSES

    private final Pose pose1 = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose pose2 = new Pose(0, 48, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private Path scorePreload;
    private PathChain pathOne;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(pose1, pose2));
//        scorePreload.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());

        /* Here is an example for Constant Interpolation
scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pathOne = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathOne);
                setPathState(1);
                break;
//            case 1:
//
//                    /* You could check for
//- Follower State: "if(!follower.isBusy()) {}"
//- Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//- Robot Position: "if(follower.getPose().getX() > 36) {}"
//*/
//
//            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//            if(!follower.isBusy()) {
//                /* Score Preload */
//
//                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                follower.followPath(grabPickup1,true);
//                setPathState(2);
//            }
//            break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(pose1);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}