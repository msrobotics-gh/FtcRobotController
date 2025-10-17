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

package org.firstinspires.ftc.teamcode;
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
@TeleOp(name = "Motor Speed with Velocity", group = "Concept")
@Config
public class LauncherDashboard extends LinearOpMode {

    public static double VELOCITY_1 = 500;
    public static double VELOCITY_2 = 500;

    double TPS_1;
    double TPS_2;


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

        motor_1 = hardwareMap.get(DcMotorEx.class, "test_1");
        motor_2 = hardwareMap.get(DcMotorEx.class, "test_2");

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        dashboardTelemetry.addData(">","press start to run motors");
        dashboardTelemetry.update();
        waitForStart();

        motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {
//            motor_1.setVelocity(VELOCITY_1);
//            motor_2.setVelocity(VELOCITY_2);

            TPS_1 = (VELOCITY_1 / 60) * 28;
            TPS_2 = (VELOCITY_2 / 60) * 28;

            motor_1.setVelocity(gamepad1.right_trigger * TPS_1);
            motor_2.setVelocity(gamepad1.left_trigger * TPS_2);


            double motor_1_velo = motor_1.getVelocity();
            double motor_2_velo = motor_2.getVelocity();

            telemetry.addData("motor 1 velocity",motor_1_velo);
            telemetry.addData("motor 2 velocity",motor_2_velo);
            telemetry.update();
            dashboardTelemetry.addData("motor 1 velocity",motor_1_velo);
            dashboardTelemetry.addData("motor 2 velocity",motor_2_velo);
            dashboardTelemetry.update();
        }

        // Turn off motor and signal done;
        motor_1.setPower(0);
        motor_2.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();
        dashboardTelemetry.addData(">","Done");
        dashboardTelemetry.update();

    }
}
