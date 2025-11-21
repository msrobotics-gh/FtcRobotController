package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous - Drive + Launch", group="Linear OpMode")
public class Auton extends LinearOpMode {

    // Drive Motors
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;

    // Intake
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;

    // Launcher
    private DcMotor upDrive = null;
    private DcMotor downDrive = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {


        leftDrive      = hardwareMap.get(DcMotor.class, "left_bottom");
        rightDrive     = hardwareMap.get(DcMotor.class, "right_bottom");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_top");
        frontRightDrive= hardwareMap.get(DcMotor.class, "right_top");

        upDrive        = hardwareMap.get(DcMotor.class, "up");
        downDrive      = hardwareMap.get(DcMotor.class, "down");

        leftIntake     = hardwareMap.get(DcMotor.class, "rightt");
        rightIntake    = hardwareMap.get(DcMotor.class, "leftt");



        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        upDrive.setDirection(DcMotor.Direction.REVERSE);
        downDrive.setDirection(DcMotor.Direction.FORWARD);


        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {


            setDrivePower(0.5, 0.5, 0.5, 0.5);
            sleep(1625);
            stopDrive();


            sleep(300);


            double launchPower = 1;

            upDrive.setPower(-1);
            downDrive.setPower(launchPower);


            sleep(1200);


            leftIntake.setPower(1.0);
            rightIntake.setPower(1.0);
            sleep(1100);
            leftIntake.setPower(1);
            rightIntake.setPower(1);
            sleep(1100);
            leftIntake.setPower(1.0);
            rightIntake.setPower(1.0);
            sleep(1000);
;

            leftIntake.setPower(0);
            rightIntake.setPower(0);
            upDrive.setPower(0);
            downDrive.setPower(0);
            stopDrive();

            setDrivePower(0, -0.75, 0.75, -0.75);
            sleep(700);
            setDrivePower(0, -0.75, 0.75, -0.75);
            sleep(750);
            stopDrive();
            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();

        }
    }


    private void setDrivePower(double lb, double rb, double lf, double rf) {
        leftDrive.setPower(lb);
        rightDrive.setPower(rb);
        frontLeftDrive.setPower(lf);
        frontRightDrive.setPower(rf);
    }


    private void stopDrive() {
        setDrivePower(0, 0, 0, 0);
    }
}
