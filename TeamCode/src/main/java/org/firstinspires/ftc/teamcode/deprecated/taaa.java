package org.firstinspires.ftc.teamcode.deprecated;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "dashboard control", group = "Deprecated")
@Config
public class taaa extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dtelemetry = dashboard.getTelemetry();
    public int x = 72;
    public int y = 72;
    public int r = 90;

    private Follower follower;

    int dontFryCpu = 0;

    @Override
    public void init() {
        telemetry.addData("> ","ready to follow");
        telemetry.update();
        dtelemetry.addData("> ","ready to follow");
        dtelemetry.update();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(x, y, Math.toRadians(r))); //set your starting pose
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        follower.update();
//        if (dontFryCpu < 10000) {
//            dontFryCpu++;
//            telemetry.addData("> ", "saving the cpu");
//            telemetry.update();
//            dtelemetry.addData("> ","saving the cpu");
//            dtelemetry.update();
//            return;
//        } else {dontFryCpu = 0;}


        final Pose tagPose = new Pose(x, y, Math.toRadians(r));
        follower.setPose(tagPose);
        telemetry.addData("> ","setting pose to (%d, %d) rotation %d",x,y,r);
        dtelemetry.addData("> ","setting pose to (%d, %d) rotation %d",x,y,r);
        telemetry.update();
        dtelemetry.update();
    }

    @Override
    public void stop() {} // dont use
}