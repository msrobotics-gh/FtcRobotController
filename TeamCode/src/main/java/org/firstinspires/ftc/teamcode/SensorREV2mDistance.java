/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates how to use the REV Robotics 2M Distance Sensor.
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.revrobotics.com/rev-31-1505/
 */
@TeleOp(name = "Sensor: REV2mDistance", group = "Sensor")
@Disabled
public class SensorREV2mDistance extends LinearOpMode {

    private DistanceSensor SensorDistance1;
    private DistanceSensor SensorDistance2;
    DcMotor conveyorMotor;
    @Override
    public void runOpMode() {
        conveyorMotor = hardwareMap.get(DcMotor.class, "left_drive");
        // you can use this as a regular DistanceSensor.
        SensorDistance1 = hardwareMap.get(DistanceSensor.class, "sensor_distance1");
        SensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_distance2");
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) SensorDistance1;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        double ConveyorSpeed = 0;
        double artifactsSeen = 0;
        boolean lockSensor = false;
        waitForStart();
        while(opModeIsActive()) {


            if (SensorDistance2.getDistance(DistanceUnit.MM) > 75 && SensorDistance1.getDistance(DistanceUnit.MM) >75) {
                ConveyorSpeed = 0.75;
            }else if (artifactsSeen % 2 != 0){
                ConveyorSpeed = 0.75;
                artifactsSeen += 1;
                lockSensor = true;
            }
            else {
                if (lockSensor == false) {
                    lockSensor = true;
                    artifactsSeen += 1;
                }
                else {
                    ConveyorSpeed = 0;
                }
            conveyorMotor.setPower(ConveyorSpeed);
            telemetry.update();
        }
    }

}
}

//by chippi chappa