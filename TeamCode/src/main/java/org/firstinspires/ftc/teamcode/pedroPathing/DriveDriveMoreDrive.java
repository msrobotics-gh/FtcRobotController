package org.firstinspires.ftc.teamcode.pedroPathing;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Auto;

import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "why do all good ftc libraries have absolutely horrendous documentation", group = "TeleOp")
@Config
public class DriveDriveMoreDrive extends NextFTCOpMode {
    public DriveDriveMoreDrive() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    public void tele(String data) {
        telemetry.addData("> ",data);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {

        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(Auto.INSTANCE.follow)
                .whenBecomesTrue(()->{ tele("following path"); });

        Gamepads.gamepad2().dpadLeft()
                .whenBecomesTrue(Auto.INSTANCE.turn)
                .whenBecomesTrue(()->{ tele("turning"); });

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(()->{ follower().breakFollowing(); })
                .whenBecomesTrue(()->{ tele("end all"); });
                // bro why do i have to use lambda expression functions for each function in pedropathing T_T i hate this library

        telemetry.addData("> ","xkcd 293");
        telemetry.update();
    }
}
