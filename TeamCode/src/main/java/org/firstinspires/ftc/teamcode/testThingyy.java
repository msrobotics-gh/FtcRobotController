package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Velauncher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Velocity Lift Drive Launch")
@Config
public class testThingyy extends NextFTCOpMode {
    public testThingyy() {
        addComponents(
                new SubsystemComponent(Lift.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    FtcDashboard dashboard = FtcDashboard.getInstance();



    // change the names and directions to suit your robot
    public  static double height;

    private IMUEx imu = new IMUEx("imu", Direction.LEFT,Direction.DOWN);

    public double topmotorpower;

    public double bottommotorpower;

    boolean isLeftBumperPressed = false;

    boolean isPressed = false;

    double startTime;
    @Override
    public void onStartButtonPressed() {



        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(new InstantCommand(()-> {
                    startTime = getRuntime();
                }))
                .whenTrue(new InstantCommand(()-> {
                    double curTime = getRuntime();
                        if (curTime - startTime > 5) {
                            CommandManager.INSTANCE.scheduleCommand(Lift.INSTANCE.toHigh);
                        }
                }));

        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(Lift.INSTANCE.toLow);

        for (String message : CommandManager.INSTANCE.snapshot()
             ) {
            telemetry.addLine(message);
        }
        telemetry.update();
    }
}
