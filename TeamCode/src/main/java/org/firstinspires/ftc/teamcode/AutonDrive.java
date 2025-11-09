package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Drivenations", group="Linear OpMode")
public class AutonDrive extends LinearOpMode {
    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private DcMotor upp = null;
    double leftPower;
    double rightPower;

    public static double speed = 0.7;
    private DcMotor upDrive = null;
    private DcMotor downDrive = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        boolean drive1 = gamepad1.y;
        boolean turn = gamepad1.y;
        boolean slow = gamepad1.b;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_bottom");
        rightDrive = hardwareMap.get(DcMotor.class, "right_bottom");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_top");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_top");
        upp = hardwareMap.get(DcMotor.class, "up");
        DcMotor leftIntake = hardwareMap.get(DcMotor.class, "rightt");
        DcMotor rightIntake = hardwareMap.get(DcMotor.class, "leftt");
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        upp.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        upDrive = hardwareMap.get(DcMotor.class, "up");
        downDrive = hardwareMap.get(DcMotor.class, "down");


        // Pushing the left stick forward MUST make robot go forward. So a       // To drive1 forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        //just these two lines based on your first test drive1.
        // Note: The settings here assume direct drive1 on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive1 wheel to save power level for telemetry
            // Choose to drive1 using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive1 straight.
            double drive2 = gamepad1.left_stick_y;
            double strafe = gamepad1.right_stick_x;
            double turn1 = -gamepad1.left_stick_x;
            double leftPower = clip(drive2 + turn1, -1, 1);
            double rightPower = clip(drive2 - turn1, -1, 1);
            double strafeLeftPower = clip(strafe, -1, 1);
            double strafeRightPower = clip(strafe, -1, 1);

            // Send calculated power to wheels
            leftDrive.setPower(leftPower - strafeLeftPower);
            rightDrive.setPower(rightPower + strafeRightPower);
            frontLeftDrive.setPower(leftPower + strafeLeftPower);
            frontRightDrive.setPower(rightPower - strafeRightPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f)", leftPower);
            telemetry.update();
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).

            // To drive1 forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive1.
            // Note: The settings here assume direct drive1 on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


            double move = 0;
            if (gamepad2.right_stick_y > 0 || gamepad2.right_stick_y < 0) {
                move = 1 * speed;
            } else if (true) {
                move = 0;
            }

            
            if (slow) {
                speed = 0.4;
            }
            else if (!slow) {
                speed = 0.7;
            }

            // Send calculated power to wheels
            leftIntake.setPower(move);
            rightIntake.setPower(move);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).


            // To drive1 forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive1.
            // Note: The settings here assume direct drive1 on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


            // Setup a variable for each drive1 wheel to save power level for telemetry
            double leftLauncher;
            double rightLauncher;

            // Choose to drive1 using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive1 straight.
            boolean drive = gamepad2.dpad_up;
            move = 0;
            if (drive == true) {
                move = 1 * speed;
            }
            else {
                move = 0;
            }
            if (gamepad2.dpad_down) {
                move = -speed;
            }



            // Send calculated power to wheels
            upDrive.setPower(-move);
            downDrive.setPower(move);



            // Send calculated power to wheels
            //  upp.setPower(gamepad2.right_stick_y*speed);
            if(gamepad2.a){
                speed=1;
            } else if (gamepad2.b) {
                speed=0.75;
            }  else if (gamepad2.y) {
                speed = 0.5;
            }  else if (gamepad2.x) {
                speed = 0.25;
            }
            else if (gamepad2.right_bumper) {

            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }
}
