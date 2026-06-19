package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="jerry go gas gas gas", group="real")
public class jerryGoGasGasGas extends LinearOpMode {
    DcMotorEx leftM;
    DcMotorEx rightM;

    @Override
    public void runOpMode() {
        leftM = hardwareMap.get(DcMotorEx.class, "l");
        rightM = hardwareMap.get(DcMotorEx.class, "r");

        leftM.setDirection(DcMotor.Direction.REVERSE);

        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "waiting");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            double leftInput = -gamepad1.left_stick_y;
            double rightInput = -gamepad1.right_stick_y;

            double MAX_TICKS_PER_SECOND = 1800.0;

            double targetVelocityL = leftInput * MAX_TICKS_PER_SECOND;
            double targetVelocityR = rightInput * MAX_TICKS_PER_SECOND;

            boolean slowMode = false;

            if (gamepad1.a){
                slowMode = true;
            }else if (gamepad1.b){
                slowMode = false;
            }

            if (slowMode){
                leftM.setVelocity(targetVelocityL/1.5);
                rightM.setVelocity(targetVelocityR/1.5);
            }else{
                leftM.setVelocity(targetVelocityL);
                rightM.setVelocity(targetVelocityR);
            }

            telemetry.addData("target L", "%5.2f", targetVelocityL);
            telemetry.addData("actual L", "%5.2f", leftM.getVelocity());
            telemetry.addData("target R", "%5.2f", targetVelocityR);
            telemetry.addData("actual R", "%5.2f", rightM.getVelocity());

            telemetry.addData("L encoder ticks", leftM.getCurrentPosition());
            telemetry.addData("R encoder ticks", rightM.getCurrentPosition());
            telemetry.update();
        }
    }
}

//The Legend of Jerry: Built to Drift
//
//Jerry didn’t start out as a legend. In the beginning, he was just a collection of aluminum channels, a scattering of loose zip ties, and a dream.
//
//Born on a cluttered workbench surrounded by the scent of warm solder, stray 3D-printer filament, and empty fast-food bags, Jerry was built with a specific purpose: to go fast. While other robots were meticulously designed to gently pick up game elements or politely navigate the field tiles, Jerry’s creators gave him a rugged, minimalist spirit. Four wheels. Two high-torque REV Robotics 20:1 HD Hex motors mounted strictly in the rear. An unpowered, lightweight front end.
//
//He was the ultimate underbird of the track—a pure, raw, rear-wheel-drive machine.
//Chapter 1: The Dark Days of Understeer
//
//In his early testing phases, Jerry was a bit of a mess. His original code was chaotic. When his drivers pushed both joysticks forward, his left motor would spin forward, but his right motor would violently reverse, causing him to spin like a top on the spot. He was blindingly fast, but he couldn't drive in a straight line.
//
//Worse, he suffered from terrible understeer. When his drivers tried to make a high-speed turn, his standard front rubber tires would dig into the foam field tiles, fighting the physics of his rear-wheel-drive setup. Instead of sliding gracefully, Jerry would blindly plow forward into the perimeter walls, his rear motors humming in frustration.
//
//Then came the battery problem. On a fresh, crisp 14.0V charge, Jerry was an absolute rocket ship, tearing across the room. But after four minutes of hard driving, his battery voltage would drop to 12.5V. Suddenly, he lost his edge. He didn't have the raw torque to break his tires loose anymore. His glorious power slides turned into sad, wide, lazy arcs.
//
//Jerry was fast, but he lacked discipline. He was a drift king without a crown.
//Chapter 2: The Digital Awakening (jerryGoGasGasGas)
//
//The turning point came when his engineers opened up Android Studio and completely rewrote his digital DNA. They named the script exactly what it was meant to be: jerryGoGasGasGas.
//
//First, they flipped his motor orientation in the software, permanently reversing the left channel so he could finally fly straight down the field. Next, they threw away standard power percentages entirely. No more open-loop guessing. They upgraded his code to DcMotorEx and unlocked his internal encoders.
//
//They programmed Jerry to understand his own physical limitations. He learned that every time his output shaft completed a full rotation, his encoders clicked exactly 560 times. He learned that his maximum free-spinning speed was 300 RPM. His engineers calculated his absolute physical threshold and locked his speed ceiling to exactly 2,500 encoder ticks per second.
//
//This changed everything. Jerry was no longer at the mercy of a dying battery. If his voltage dropped, his internal PID loops automatically pumped more juice to his coils to compensate, keeping his speed perfectly consistent from the first second of the match to the very last. Finally, they activated his BRAKE zero-power behavior, giving him a digital handbrake that could snap him back into a straight line on a dime.
//Chapter 3: The King of the Foam Tiles
//
//The first time Jerry ran the fully optimized jerryGoGasGasGas script, the workshop went dead silent.
//
//Jerry accelerated down the center lane, his REV 20:1 motors whining in perfect harmony at a flawless 2,500 ticks per second. As he approached the corner, his driver executed the ultimate tank-stick maneuver: keeping the left joystick pinned completely forward while violently slamming the right joystick into a full reversal.
//
//The Control Hub responded instantly. It flooded Jerry's left rear motor with maximum voltage to maintain its forward velocity while forcing the right rear motor into a violent, backwards spin.
//
//With his lightweight front end pivoting effortlessly, Jerry’s rear rubber tires finally broke traction. The tail of the robot swung out wide in a magnificent, sweeping, perfectly controlled power slide. He drifted around the corner, skimming millimeters from the field elements, completely sideways, before the driver let go of the sticks. Jerry's digital handbrake engaged, his tires bit back into the foam, and he shot forward in a straight line like a arrow.
//
//Today, Jerry resides in the team's tech lab, safely cached in the local Gradle repository. He doesn't need the internet, and he doesn't care about network proxies. As long as his code is compiled and his rear wheels are spinning, Jerry is ready to gas, gas, gas.