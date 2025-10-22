package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelGate;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "ServoGateOpMode")
public class ServoGateOpMode extends NextFTCOpMode {


    public ServoGateOpMode() {
        addComponents(new SubsystemComponent(FlywheelGate.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
                );
    }

    @Override
    public void onUpdate(){
        BindingManager.update();
    }

    @Override
    public void onStop(){
        BindingManager.reset();
    }

    @Override
    public void onStartButtonPressed() {

        Button gamepad2DpadUp = button(() -> gamepad2.dpad_up);
        gamepad2DpadUp.whenBecomesTrue(FlywheelGate.INSTANCE::open);

        Button gamepad2DpadDown = button(() -> gamepad2.dpad_down);
        gamepad2DpadDown.whenBecomesTrue(FlywheelGate.INSTANCE::close);



    }
}
