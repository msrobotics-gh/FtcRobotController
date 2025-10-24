/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.temp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a Robot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "ViperSlide Lift Control Using D-Pad", group = "Concept")
@Config
public class ViperslideGamepad extends LinearOpMode {

    public static double POWER = 0.5;

    public static int MOTOR_1_MULT = -1;
    public static int MOTOR_2_MULT = -1;


    // Define class members
    DcMotorEx motor_1;
    DcMotorEx motor_2;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() {

//        !! IMPORTANT !!
//        set first motor name to "motor"
//        set second motor name to "lowter"

        motor_1 = hardwareMap.get(DcMotorEx.class, "fastboi"); // 0
        motor_2 = hardwareMap.get(DcMotorEx.class, "slowboi"); // 1

        // Wait for the start button
        telemetry.addData("> ", "press start pls ty" );
        telemetry.update();
        dashboardTelemetry.addData("> ","press start pls ty");
        dashboardTelemetry.update();
        waitForStart();

        motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {
//            motor_1.setVelocity(VELOCITY_1);
//            motor_2.setVelocity(VELOCITY_2);

            if (gamepad1.dpad_up) {
                motor_1.setPower(POWER * MOTOR_1_MULT);
                motor_2.setPower(POWER * MOTOR_2_MULT);
                telemetry.addData("> ","goin up");
                dashboardTelemetry.addData("> ","goin up");
            } else if (gamepad1.dpad_down) {
                motor_1.setPower(POWER * -MOTOR_1_MULT);
                motor_2.setPower(POWER * -MOTOR_2_MULT);
                telemetry.addData("> ","goin down");
                dashboardTelemetry.addData("> ","goin down");
            } else {
                motor_1.setPower(0);
                motor_2.setPower(0);
                telemetry.addData("> ", "chillin");
                dashboardTelemetry.addData("> ","chillin");
            }
            telemetry.update();
            dashboardTelemetry.update();
        }

        // Turn off motor and signal done;
        motor_1.setPower(0);
        motor_2.setPower(0);
        telemetry.addData("> ", "finished");
        telemetry.update();
        dashboardTelemetry.addData("> ","finsihed");
        dashboardTelemetry.update();

    }
}
