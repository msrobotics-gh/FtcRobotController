package org.firstinspires.ftc.teamcode.temp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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
public class meese extends NextFTCOpMode {
    public meese() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(FlywheelGate.INSTANCE),
                new SubsystemComponent(Velauncher.INSTANCE)
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
    @Override
    public void onStartButtonPressed() {


        Gamepads.gamepad2().leftBumper()
                .whenTrue(Velauncher.INSTANCE.velaunch)
                .whenBecomesFalse(Velauncher.INSTANCE.unvelaunch);


            Gamepads.gamepad2().rightBumper()
                    .whenBecomesTrue(FlywheelGate.INSTANCE.open())
                    .whenBecomesTrue(Intake.INSTANCE.intake)
                    .whenBecomesTrue(Intake.INSTANCE.intakesecond)
                    .whenBecomesFalse(Intake.INSTANCE.intakeoff)
                    .whenBecomesFalse(Intake.INSTANCE.intakeoff2)
                    .whenBecomesFalse(FlywheelGate.INSTANCE.close());

        Gamepads.gamepad2().a()
                .whenBecomesTrue(Intake.INSTANCE.intake)
                .whenBecomesTrue(Intake.INSTANCE.intakesecond);

        Gamepads.gamepad2().b()
                .whenBecomesTrue(Intake.INSTANCE.intakeoff)
                .whenBecomesTrue(Intake.INSTANCE.intakeoff2);

        for (String message : CommandManager.INSTANCE.snapshot()
             ) {
            telemetry.addLine(message);
        }
        telemetry.update();
    }
}
