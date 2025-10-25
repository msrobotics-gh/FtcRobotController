package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
//import org.firstinspires.ftc.teamcode.subsystems.Velauncher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "lift drive launch")
@Config
public class LiftDriveLaunch extends NextFTCOpMode {
    public LiftDriveLaunch() {
        addComponents(
                new SubsystemComponent(Lift.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(FlywheelGate.INSTANCE)
        );
    }
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("front_left");
    private final MotorEx frontRightMotor = new MotorEx("front_right").reversed();
    private final MotorEx backLeftMotor = new MotorEx("back_left").reversed();
    private final MotorEx backRightMotor = new MotorEx("back_right").reversed();
    public  static double height;

    public double topmotorpower;

    public double bottommotorpower;

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(Lift.INSTANCE.toHigh);

        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(Lift.INSTANCE.toLow);

        Gamepads.gamepad2().leftBumper()
//                .whenTrue(Velauncher.INSTANCE.velaunch)
//                .whenTrue(Velauncher.INSTANCE.velaunch2)
                .whenBecomesFalse(Launcher.INSTANCE.unspinflywheel)
                .whenBecomesFalse(Launcher.INSTANCE.unspinflywheel2);


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
