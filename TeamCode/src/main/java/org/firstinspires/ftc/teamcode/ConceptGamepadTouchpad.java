package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name="Concept: Gamepad Touchpad", group ="Concept")
public class ConceptGamepadTouchpad extends LinearOpMode
{

    DcMotor lift1;
    DcMotor lift2;
    int power = 1;
    @Override
    public void runOpMode()
    {

        lift1 = hardwareMap.get(DcMotor.class,"lift1");
        lift2 = hardwareMap.get(DcMotor.class,"lift2");
        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.dpad_up)
            {
                lift1.setPower(power);
                lift2.setPower(power);

            }else if(gamepad1.dpad_down) {

                lift1.setPower(-1 * power);
                lift2.setPower(-1 * power);

            }else {
                lift1.setPower(0);
                lift2.setPower(0);
            }

        }
    }
}
