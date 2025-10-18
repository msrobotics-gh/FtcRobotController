/* Copyright (c) 2077 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */








package org.firstinspires.ftc.teamcode.Mechanisms;








import static com.qualcomm.robotcore.util.Range.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
















/*
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */








@TeleOp(name="Drivenations", group="Linear OpMode")
public class Teleop_26266_2025_2026 extends LinearOpMode {


    // Declare OpMode members.

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    double leftPower;
    double rightPower;
    boolean press = false;
    boolean drive1 = gamepad1.y;
    boolean turn = gamepad1.y;
    public static double speed = 0.7;
    private DcMotor leftLauncher = null;
    private DcMotor rightLauncher = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_bottom");
        rightDrive = hardwareMap.get(DcMotor.class, "right_bottom");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_top");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_top");


        // Pushing the left stick forward MUST make robot go forward. So a       // To drive1 forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        //djust these two lines based on your first test drive1.
        // Note: The settings here assume direct drive1 on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
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
            double drive2 = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double leftPower = clip(drive2 + turn, -0.4, 0.4);
            double rightPower = clip(drive2 - turn, -0.4, 0.4);
            double strafeLeftPower = clip(strafe, -0.4, 0.4);
            double strafeRightPower = clip(strafe, -0.4, 0.4);


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive1 forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;


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
            DcMotor leftIntake = hardwareMap.get(DcMotor.class, "rightt");
            DcMotor rightIntake = hardwareMap.get(DcMotor.class, "leftt");

            // To drive1 forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive1.
            // Note: The settings here assume direct drive1 on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            leftIntake.setDirection(DcMotor.Direction.REVERSE);
            rightIntake.setDirection(DcMotor.Direction.FORWARD);

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

                double move = 0;
                if (drive2 > 0 || drive2 < 0) {
                    if (press == true) {
                        press = false;
                    } else {
                        press = true;
                    }

                    move = 1 * speed;
                } else if (true) {
                    move = 0;
                }

                //leftPower    = Range.clip(drive1 + turn, -speed, speed) ;
                //rightPower   = Range.clip(drive1 - turn, -speed, speed) ;

                // Tank Mode uses one stick to control each wheel.
                // - This requires no math, but it is hard to drive1 forward slowly and keep straight.
                // leftPower  = -gamepad1.left_stick_y ;
                // rightPower = -gamepad1.right_stick_y ;

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
                leftDrive = hardwareMap.get(DcMotor.class, "up");
                rightDrive = hardwareMap.get(DcMotor.class, "down");

                // To drive1 forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
                // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive1.
                // Note: The settings here assume direct drive1 on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
                leftDrive.setDirection(DcMotor.Direction.FORWARD);
                rightDrive.setDirection(DcMotor.Direction.FORWARD);

                // Wait for the game to start (driver presses START)
                waitForStart();
                runtime.reset();

                // run until the end of the match (driver presses STOP)
                while (opModeIsActive()) {

                    // Setup a variable for each drive1 wheel to save power level for telemetry
                    double leftLauncher;
                    double rightLauncher;

                    // Choose to drive1 using either Tank Mode, or POV Mode
                    // Comment out the method that's not used.  The default below is POV.

                    // POV Mode uses left stick to go forward, and right stick to turn.
                    // - This uses basic math to combine motions and is easier to drive1 straight.
                    boolean press = false;
                    boolean drive = gamepad1.a;
                    move = 0;
                    if (drive == true) {
                        if (press == true) {
                            press = false;
                        } else {
                            press = true;
                        }

                        move = 1 * speed;
                    } else if (true) {
                        move = 0;
                    }

                    //leftPower    = Range.clip(drive1 + turn, -speed, speed) ;
                    //rightPower   = Range.clip(drive1 - turn, -speed, speed) ;

                    // Tank Mode uses one stick to control each wheel.
                    // - This requires no math, but it is hard to drive1 forward slowly and keep straight.
                    // leftPower  = -gamepad1.left_stick_y ;
                    // rightPower = -gamepad1.right_stick_y ;

                    // Send calculated power to wheels
                    leftDrive.setPower(move);
                    rightDrive.setPower(move);


                }
            }

        }


    }
}
