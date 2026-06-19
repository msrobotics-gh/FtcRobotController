package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "jerry go gas gas gas", group = "real")

public class jerryGoGasGasGas extends LinearOpMode {

    // Define class members
    DcMotor leftM;
    DcMotor rightM;

    @Override
    public void runOpMode() {
        leftM = hardwareMap.get(DcMotor.class, "l");
        rightM = hardwareMap.get(DcMotor.class, "r");

        telemetry.addData(">", "waiting" );
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            double motorPowerL = gamepad1.left_stick_y;
            double motorPowerR = gamepad1.right_stick_y;

            telemetry.addData("Motor Power L", "%5.2f", motorPowerL);
            telemetry.addData("Motor Power R", "%5.2f", motorPowerR);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            leftM.setPower(motorPowerL);
            rightM.setPower(-motorPowerR);
        }

        telemetry.addData(">", "done");
        telemetry.update();

    }
}
