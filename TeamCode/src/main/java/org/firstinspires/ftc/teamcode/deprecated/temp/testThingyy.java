//package org.firstinspires.ftc.teamcode.temp;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.subsystems.Lift;
//
//import dev.nextftc.core.commands.CommandManager;
//import dev.nextftc.core.commands.utility.InstantCommand;
//import dev.nextftc.core.components.BindingsComponent;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.ftc.Gamepads;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.ftc.components.BulkReadComponent;
//import dev.nextftc.hardware.impl.Direction;
//import dev.nextftc.hardware.impl.IMUEx;
//
//@TeleOp(name = "Lift Run Test")
//@Config
//public class testThingyy extends NextFTCOpMode {
//    public testThingyy() {
//        addComponents(
//                new SubsystemComponent(Lift.INSTANCE),
//                BulkReadComponent.INSTANCE,
//                BindingsComponent.INSTANCE
//        );
//    }
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//
//
//
//    // change the names and directions to suit your robot
//    public  static double height;
//
//    private IMUEx imu = new IMUEx("imu", Direction.LEFT,Direction.DOWN);
//
//    public double topmotorpower;
//
//    public double bottommotorpower;
//
//    boolean isLeftBumperPressed = false;
//
//    boolean isPressed = false;
//
//    double startTime;
//    @Override
//    public void onStartButtonPressed() {
//
//
//
//        Gamepads.gamepad2().dpadUp()
//                .whenBecomesTrue(Lift.INSTANCE.toPos)
//                .whenBecomesTrue(Lift.INSTANCE.toPos2)
//                .whenBecomesFalse(Lift.INSTANCE.stop)
//                .whenBecomesFalse(Lift.INSTANCE.stop2);
//
//
//        }
//    }
