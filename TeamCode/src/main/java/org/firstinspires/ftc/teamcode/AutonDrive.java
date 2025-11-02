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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
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
@Autonomous(name="Drive", group="Linear OpMode")
public class AutonDrive extends LinearOpMode {
    // Declare OpMode members.
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;

    private double drive = -1.0;
    private double strafe = 0.0;
    private double turn  = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    public double leftPower;
    public double rightPower;
    public double strafeLeftPower;
    public double strafeRightPower;

    private static final double COUNTS_PER_MOTOR_REV = 28.0; // Example for HD Hex Motor, adjust for your motor
    private static final double MAX_POWER = 0.4;

    public void resetRotations() {
        if (leftDrive == null || rightDrive == null || frontLeftDrive == null || frontRightDrive == null) {
            return;
        }

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double speed) {

    }
    private double getRotations(DcMotor motor) {
        return Math.abs(motor.getCurrentPosition()) / COUNTS_PER_MOTOR_REV;
    }

    private void stopAllDriveMotors() {
        if (leftDrive != null) {
            leftDrive.setPower(0);
        }
        if (rightDrive != null) {
            rightDrive.setPower(0);
        }
        if (frontLeftDrive != null) {
            frontLeftDrive.setPower(0);
        }
        if (frontRightDrive != null) {
            frontRightDrive.setPower(0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_bottom");
        rightDrive = hardwareMap.get(DcMotor.class, "right_bottom");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_top");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_top");

        // Pushing the left stick forward MUST make robot go forward. So a       // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // just these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        resetRotations();
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            /*
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;
            */
            //double drive = -1;
            //double strafe = gamepad1.left_stick_x;
            //double turn  =  gamepad1.right_stick_x;

            leftPower    = Range.clip(drive + turn, -MAX_POWER, MAX_POWER);
            rightPower   = Range.clip(drive - turn, -MAX_POWER, MAX_POWER);
            strafeLeftPower = Range.clip(strafe, -MAX_POWER, MAX_POWER);
            strafeRightPower = Range.clip(strafe, -MAX_POWER, MAX_POWER);
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            driveForRotations(3);

            // Show the elapsed game timer and wheel power.
            telemetry.addData("Status", "Run Time: ");
            telemetry.addData("Motors", "left (%.2f)", leftPower);
            telemetry.update();
        }
    }
    public void driveForRotations(double rotations) {
        if (!opModeIsActive()) {
            return;
        }

        double backLeftCommand = leftPower - strafeLeftPower;
        double backRightCommand = rightPower + strafeRightPower;
        double frontLeftCommand = leftPower + strafeLeftPower;
        double frontRightCommand = rightPower - strafeRightPower;

        if (Math.abs(backLeftCommand) < 1e-4
                && Math.abs(backRightCommand) < 1e-4
                && Math.abs(frontLeftCommand) < 1e-4
                && Math.abs(frontRightCommand) < 1e-4) {
            return;
        }

        resetRotations();

        leftDrive.setPower(backLeftCommand);
        rightDrive.setPower(backRightCommand);
        frontLeftDrive.setPower(frontLeftCommand);
        frontRightDrive.setPower(frontRightCommand);

        while (opModeIsActive()
                && getRotations(leftDrive) < rotations
                && getRotations(rightDrive) < rotations
                && getRotations(frontLeftDrive) < rotations
                && getRotations(frontRightDrive) < rotations) {
            idle();
        }

        stopAllDriveMotors();
        drive = 0;
    }
    public void turnForRotation(double rotations, double direction) throws InterruptedException {
        if (!opModeIsActive()) {
            return;
        }

        double turnPower = Range.clip(direction, -MAX_POWER, MAX_POWER);
        if (turnPower == 0) {
            return;
        }

        resetRotations();

        leftDrive.setPower(turnPower);
        rightDrive.setPower(-turnPower);
        frontLeftDrive.setPower(turnPower);
        frontRightDrive.setPower(-turnPower);

        while (opModeIsActive()
                && getRotations(leftDrive) < rotations
                && getRotations(rightDrive) < rotations
                && getRotations(frontLeftDrive) < rotations
                && getRotations(frontRightDrive) < rotations) {
            idle();
        }

        stopAllDriveMotors();
        turn = 0;
    }
    public void strafeForRotation(double rotations) {
        double directionPower = strafeRightPower;
        if (directionPower == 0) {
            directionPower = MAX_POWER;
        }
        strafeForRotation(rotations, directionPower);
    }
    public void strafeForRotation(double rotations, double direction) {
        if (!opModeIsActive()) {
            return;
        }

        double strafePower = Range.clip(direction, -MAX_POWER, MAX_POWER);
        if (strafePower == 0) {
            return;
        }

        resetRotations();

        leftDrive.setPower(-strafePower);
        rightDrive.setPower(strafePower);
        frontLeftDrive.setPower(strafePower);
        frontRightDrive.setPower(-strafePower);

        while (opModeIsActive()
                && getRotations(leftDrive) < rotations
                && getRotations(rightDrive) < rotations
                && getRotations(frontLeftDrive) < rotations
                && getRotations(frontRightDrive) < rotations) {
            idle();
        }

        stopAllDriveMotors();
        strafe = 0;
    }

    }
