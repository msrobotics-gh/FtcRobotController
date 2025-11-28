package org.firstinspires.ftc.teamcode.temp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Velauncher;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

@TeleOp(name = "Velocity Lift Drive Launch Meese")
@Config
@Disabled
public class meese extends NextFTCOpMode {
    public meese() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Lift.INSTANCE)
        );
    }
    FtcDashboard dashboard = FtcDashboard.getInstance();



    // change the names and directions to suit your robot
    public  static double height;

    private IMUEx imu = new IMUEx("imu", Direction.LEFT,Direction.DOWN);

    boolean isLeftBumperPressed = false;

    boolean isPressed = false;
    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(Lift.INSTANCE.liftHigh);

        for (String message : CommandManager.INSTANCE.snapshot()
             ) {
            telemetry.addLine(message);
        }
        telemetry.update();
    }
}
