// IMPORT STUFF
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

// *** NEW: AprilTag Imports ***
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

// BLUE ALLIANCE!!!!!!!!!!
// ADDING STUFF
@Autonomous(name = "Auton Drive", group = "Concept")
public class BLUEbottomAutonTest20 extends LinearOpMode {

    // MOTORS
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor IntakeMotor = null;
    private DcMotor ConveyorMotor = null;
    private DcMotor LaunchMotorTOP = null;
    private DcMotor LaunchMotorBOTTOM = null;
    private DistanceSensor SensorDistance1 = null;
    private DistanceSensor SensorDistance2 = null;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // TIME
    private ElapsedTime runtime = new ElapsedTime();


    // CONSTANTS
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 3.0;
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);

    // SPEEDS
    static final double DRIVE_SPEED = 0.5;
    static final double INTAKE_SPEED = 0.7;
    static final double CONVEYOR_SPEED = 0.8;


    // FIXXXXX
    static final double ROBOT_TRACK_WIDTH_INCHES = 9.0; // IMPORTANT: Measure the distance between left and right wheels
    static final double TURN_DEGREES_TO_INCHES = (ROBOT_TRACK_WIDTH_INCHES * 3.1415) / 360;


    @Override
    public void runOpMode() {

        // CONFIGURATION NAMES
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        BackRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        IntakeMotor = hardwareMap.get(DcMotor.class, "intake");
        ConveyorMotor = hardwareMap.get(DcMotor.class, "conveyor");
        SensorDistance1 = hardwareMap.get(DistanceSensor.class, "sensor_distance1");
        SensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_distance2");
        LaunchMotorTOP = hardwareMap.get(DcMotor.class, "launch_top");
        LaunchMotorBOTTOM = hardwareMap.get(DcMotor.class, "launch_bottom");

        // APRILTAG INITIALIZATION
        initAprilTag();

        // MOTOR DIRECTIONS
        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // ENCODER STUFF
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // (Default to -1, meaning no tag seen)
        int detectedTagID = -1;

        // APRILTAG LOOP
        while (!isStarted() && !isStopRequested()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    // Store the ID of the detected tag
                    detectedTagID = detection.id;
                }
            }
            telemetry.addData("Decision", "Will run path for ID: " + detectedTagID);
            telemetry.update();
            sleep(20);
        }


        waitForStart();

        // MAIN CODE
        if (opModeIsActive()) {
            encoderStrafe(DRIVE_SPEED, -12, 10.0);
            sleep(1000);

            // GPP
            if (detectedTagID == 21) {
                // Step 1: Strafe LEFT
                encoderStrafe(DRIVE_SPEED, -42, 10.0);
                sleep(1000);

                // Step 2: Drive FORWARD to REACH ARTIFACT 1
                encoderDrive(DRIVE_SPEED, 36, 10.0);
                sleep(1000);

                // Step 3: Intake movement
                IntakeMotor.setPower(INTAKE_SPEED);

                // Step 4: Conveyor movement
                ConveyorMotor.setPower(CONVEYOR_SPEED);

                // Step 5: Drive FORWARD to GET ARTIFACT 1
                encoderDrive(DRIVE_SPEED, 2, 10.0);
                sleep(2000);

               // Step 6: Drive FORWARD to REACH ARTIFACT 2
                encoderDrive(DRIVE_SPEED, 10, 10);
                sleep(1000);

                // Step 7: Drive FORWARD to GET ARTIFACT 2
                encoderDrive(DRIVE_SPEED, 2, 10);
                sleep(2000);

                // Step 8: Stop intake and conveyor
                IntakeMotor.setPower(0);
                ConveyorMotor.setPower(0);
                sleep(2000);

                // Step 9: Strafe RIGHT
                encoderStrafe(DRIVE_SPEED, 42, 10.0);
                sleep(1000);

                // Step 10: Drive FORWARD to launch position
                encoderDrive(DRIVE_SPEED, 10, 10.0);
                sleep(1000);

                // Step 11: Rotate LEFT
                encoderTurn(DRIVE_SPEED, 45, 10.0);
                sleep(1000);

                // Step 12: Launching
                LaunchMotorTOP.setPower(0.4);
                LaunchMotorBOTTOM.setPower(0.4);
                sleep(100);
                ConveyorMotor.setPower(1);
                sleep(10000);

                // Step 13: Stop everything
                LaunchMotorTOP.setPower(0);
                LaunchMotorBOTTOM.setPower(0);
                ConveyorMotor.setPower(0);

                // Step 14: Rotate right 45 degrees RIGHT
                encoderTurn(DRIVE_SPEED, -45, 10.0);
                sleep(1000);

                // Step 15: Strafe LEFT 42 inches
                encoderStrafe(DRIVE_SPEED, 42, 10.0);
                sleep(1000);

                //Step 16: Intake movement
                IntakeMotor.setPower(INTAKE_SPEED);

                // Step 17: Conveyor movement
                ConveyorMotor.setPower(CONVEYOR_SPEED);

                // Step 18: Drive FORWARD to REACH ARTIFACT 3
                encoderDrive(DRIVE_SPEED, 12, 10.0);

                // Step 19: Drive FORWARD to GET ARTIFACT 3
                encoderDrive(DRIVE_SPEED, 2, 10.0);

                // Step 20: Strafe RIGHT 42 inches
                encoderStrafe(DRIVE_SPEED, -42, 10.0);

                // Step 21: Go BACKWARD 14 inches to launch position
                encoderDrive(DRIVE_SPEED, -14, 10.0);

                // Step 22: Rotate LEFT 45 degrees
                encoderTurn(DRIVE_SPEED, 45, 10.0);

                // Step 23: Launching
                LaunchMotorTOP.setPower(0.4);
                LaunchMotorBOTTOM.setPower(0.4);
                sleep(100);
                ConveyorMotor.setPower(1);
                sleep(10000);

                // Step 24: Stop everything
                LaunchMotorTOP.setPower(0);
                LaunchMotorBOTTOM.setPower(0);
                ConveyorMotor.setPower(0);

                // Step 25: Rotate 45 degrees RIGHT
                encoderTurn(DRIVE_SPEED, -45, 10.0);

                // Step 26: Get off the launch zone
                encoderDrive(DRIVE_SPEED, -5, 10.0);

            }
            // PGP
            else if (detectedTagID == 22) {
                // Step 1: Strafe LEFT
                encoderStrafe(DRIVE_SPEED, -48, 10.0);
                sleep(1000);

                // Step 2: Drive FORWARD to REACH ARTIFACT 1
                encoderDrive(DRIVE_SPEED, 36, 10.0);
                sleep(1000);

                // Step 3: Intake movement
                IntakeMotor.setPower(INTAKE_SPEED);

                // Step 4: Conveyor movement
                ConveyorMotor.setPower(CONVEYOR_SPEED);

                // Step 5: Drive FORWARD to GET ARTIFACT 1
                encoderDrive(DRIVE_SPEED, 2, 10.0);
                sleep(2000);

                // Step 6: Drive FORWARD to REACH ARTIFACT 2
                encoderDrive(DRIVE_SPEED, 10, 10);
                sleep(1000);

                // Step 7: Drive FORWARD to GET ARTIFACT 2
                encoderDrive(DRIVE_SPEED, 2, 10);
                sleep(2000);

                // Step 8: Stop intake and conveyor
                IntakeMotor.setPower(0);
                ConveyorMotor.setPower(0);
                sleep(2000);

                // Step 9: Strafe RIGHT
                encoderStrafe(DRIVE_SPEED, 48, 10.0);
                sleep(1000);

                // Step 10: Drive FORWARD to launch position
                encoderDrive(DRIVE_SPEED, 10, 10.0);
                sleep(1000);

                // Step 11: Rotate LEFT
                encoderTurn(DRIVE_SPEED, 45, 10.0);
                sleep(1000);

                // Step 12: Launching
                LaunchMotorTOP.setPower(0.4);
                LaunchMotorBOTTOM.setPower(0.4);
                sleep(100);
                ConveyorMotor.setPower(1);
                sleep(10000);

                // Step 13: stop everything
                LaunchMotorTOP.setPower(0);
                LaunchMotorBOTTOM.setPower(0);
                ConveyorMotor.setPower(0);

                // Step 14: Rotate right 45 degrees RIGHT
                encoderTurn(DRIVE_SPEED, -45, 10.0);
                sleep(1000);

                // Step 15: Strafe LEFT 42 inches
                encoderStrafe(DRIVE_SPEED, 48, 10.0);
                sleep(1000);

                //Step 16: Intake movement
                IntakeMotor.setPower(INTAKE_SPEED);

                // Step 17: Conveyor movement
                ConveyorMotor.setPower(CONVEYOR_SPEED);

                // Step 18: Drive FORWARD to REACH ARTIFACT 3
                encoderDrive(DRIVE_SPEED, 12, 10.0);

                // Step 19: Drive FORWARD to GET ARTIFACT 3
                encoderDrive(DRIVE_SPEED, 2, 10.0);

                // Step 20: Strafe RIGHT 42 inches
                encoderStrafe(DRIVE_SPEED, -48, 10.0);

                // Step 21: Go BACKWARD 14 inches to launch position
                encoderDrive(DRIVE_SPEED, -14, 10.0);

                // Step 22: Rotate LEFT 45 degrees
                encoderTurn(DRIVE_SPEED, 45, 10.0);

                // Step 23: Launching
                LaunchMotorTOP.setPower(0.4);
                LaunchMotorBOTTOM.setPower(0.4);
                sleep(100);
                ConveyorMotor.setPower(1);
                sleep(10000);

                // Step 24: Stop everything
                LaunchMotorTOP.setPower(0);
                LaunchMotorBOTTOM.setPower(0);
                ConveyorMotor.setPower(0);

                // Step 25: Rotate 45 degrees RIGHT
                encoderTurn(DRIVE_SPEED, -45, 10.0);

                // Step 26: Get off the launch zone
                encoderDrive(DRIVE_SPEED, -5, 10.0);
            }
            // PPG
            else if (detectedTagID == 23){
                // Step 1: Strafe LEFT
                encoderStrafe(DRIVE_SPEED, -54, 10.0);
                sleep(1000);

                // Step 2: Drive FORWARD
                encoderDrive(DRIVE_SPEED, 36, 10.0);
                sleep(1000);

                // Step 3: Intake movement
                IntakeMotor.setPower(INTAKE_SPEED);

                // Step 4: Conveyor movement
                ConveyorMotor.setPower(CONVEYOR_SPEED);

                // Step 5: Drive FORWARD to GET ARTIFACT 1
                encoderDrive(DRIVE_SPEED, 2, 10.0);
                sleep(2000);

                // Step 6: Drive FORWARD to REACH ARTIFACT 2
                encoderDrive(DRIVE_SPEED, 10, 10);
                sleep(1000);

                // Step 7: Drive FORWARD to GET ARTIFACT 2
                encoderDrive(DRIVE_SPEED, 2, 10);
                sleep(2000);

                // Step 8: Stop intake and conveyor
                IntakeMotor.setPower(0);
                ConveyorMotor.setPower(0);
                sleep(2000);

                // Step 9: Strafe RIGHT
                encoderStrafe(DRIVE_SPEED, 54, 10.0);
                sleep(1000);

                // Step 10: Drive FORWARD to launch position
                encoderDrive(DRIVE_SPEED, 10, 10.0);
                sleep(1000);

                // Step 11: Rotate LEFT
                encoderTurn(DRIVE_SPEED, 45, 10.0);
                sleep(1000);

                // Step 12: Launching
                LaunchMotorTOP.setPower(0.4);
                LaunchMotorBOTTOM.setPower(0.4);
                sleep(100);
                ConveyorMotor.setPower(1);
                sleep(10000);

                // Step 13: Stop everything
                LaunchMotorTOP.setPower(0);
                LaunchMotorBOTTOM.setPower(0);
                ConveyorMotor.setPower(0);

                // Step 14: Rotate right 45 degrees RIGHT
                encoderTurn(DRIVE_SPEED, -45, 10.0);
                sleep(1000);

                // Step 15: Strafe LEFT 42 inches
                encoderStrafe(DRIVE_SPEED, 54, 10.0);
                sleep(1000);

                //Step 16: Intake movement
                IntakeMotor.setPower(INTAKE_SPEED);

                // Step 17: Conveyor movement
                ConveyorMotor.setPower(CONVEYOR_SPEED);

                // Step 18: Drive FORWARD to REACH ARTIFACT 3
                encoderDrive(DRIVE_SPEED, 12, 10.0);

                // Step 19: Drive FORWARD to GET ARTIFACT 3
                encoderDrive(DRIVE_SPEED, 2, 10.0);

                // Step 20: Strafe RIGHT 42 inches
                encoderStrafe(DRIVE_SPEED, -54, 10.0);

                // Step 21: Go BACKWARD 14 inches to launch position
                encoderDrive(DRIVE_SPEED, -14, 10.0);

                // Step 22: Rotate LEFT 45 degrees
                encoderTurn(DRIVE_SPEED, 45, 10.0);

                // Step 23: Launching
                LaunchMotorTOP.setPower(0.4);
                LaunchMotorBOTTOM.setPower(0.4);
                sleep(100);
                ConveyorMotor.setPower(1);
                sleep(10000);

                // Step 24: Stop everything
                LaunchMotorTOP.setPower(0);
                LaunchMotorBOTTOM.setPower(0);
                ConveyorMotor.setPower(0);

                // Step 25: Rotate 45 degrees RIGHT
                encoderTurn(DRIVE_SPEED, -45, 10.0);

                // Step 26: Get off the launch zone
                encoderDrive(DRIVE_SPEED, -5, 10.0);
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }


    /**
     * Initializes the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the vision portal by combining the camera and the processor.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }


    // ENCODER FUNCTIONS
    public void encoderStrafe(double speed, double inches, double timeoutS) {
        if (opModeIsActive()) {
            int target = (int) (inches * COUNTS_PER_INCH);

            // For strafing, the front-left/back-right pair is opposite to the front-right/back-left pair.
            FrontLeftDrive.setTargetPosition(FrontLeftDrive.getCurrentPosition() + target);
            FrontRightDrive.setTargetPosition(FrontRightDrive.getCurrentPosition() - target);
            BackLeftDrive.setTargetPosition(BackLeftDrive.getCurrentPosition() - target);
            BackRightDrive.setTargetPosition(BackRightDrive.getCurrentPosition() + target);

            // Set mode and power
            FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            FrontLeftDrive.setPower(Math.abs(speed));
            FrontRightDrive.setPower(Math.abs(speed));
            BackLeftDrive.setPower(Math.abs(speed));
            BackRightDrive.setPower(Math.abs(speed));

            // Wait until done or timed out
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (FrontLeftDrive.isBusy() && FrontRightDrive.isBusy() && BackLeftDrive.isBusy() && BackRightDrive.isBusy())) {
                // Wait for the movement to finish.
            }

            // Stop all motion
            FrontLeftDrive.setPower(0);
            FrontRightDrive.setPower(0);
            BackLeftDrive.setPower(0);
            BackRightDrive.setPower(0);

            // Reset modes for next command
            FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void encoderDrive(double speed, double inches, double timeoutS) { if (opModeIsActive()) {
        int target = (int) (inches * COUNTS_PER_INCH);

        // For driving forward, all motors receive the same target command.
        FrontLeftDrive.setTargetPosition(FrontLeftDrive.getCurrentPosition() + target);
        FrontRightDrive.setTargetPosition(FrontRightDrive.getCurrentPosition() + target);
        BackLeftDrive.setTargetPosition(BackLeftDrive.getCurrentPosition() + target);
        BackRightDrive.setTargetPosition(BackRightDrive.getCurrentPosition() + target);

        // Set mode and power
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        FrontLeftDrive.setPower(Math.abs(speed));
        FrontRightDrive.setPower(Math.abs(speed));
        BackLeftDrive.setPower(Math.abs(speed));
        BackRightDrive.setPower(Math.abs(speed));

        // Wait until done or timed out
        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (FrontLeftDrive.isBusy() && FrontRightDrive.isBusy() && BackLeftDrive.isBusy() && BackRightDrive.isBusy())) {
            // Wait for the movement to finish. Telemetry could be added here for debugging.
        }

        // Stop all motion
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);

        // Reset modes for next command
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    }
    public void encoderTurn(double speed, double degrees, double timeoutS) { if (opModeIsActive()) {
        // To turn LEFT (positive degrees), left wheels go backward, right wheels go forward.
        double inches = degrees * TURN_DEGREES_TO_INCHES;
        int leftTarget = (int) (-inches * COUNTS_PER_INCH);  // Left side gets negative target
        int rightTarget = (int) (inches * COUNTS_PER_INCH); // Right side gets positive target

        // Set the target positions for each side
        FrontLeftDrive.setTargetPosition(FrontLeftDrive.getCurrentPosition() + leftTarget);
        BackLeftDrive.setTargetPosition(BackLeftDrive.getCurrentPosition() + leftTarget);
        FrontRightDrive.setTargetPosition(FrontRightDrive.getCurrentPosition() + rightTarget);
        BackRightDrive.setTargetPosition(BackRightDrive.getCurrentPosition() + rightTarget);

        // Set mode and power
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        FrontLeftDrive.setPower(Math.abs(speed));
        FrontRightDrive.setPower(Math.abs(speed));
        BackLeftDrive.setPower(Math.abs(speed));
        BackRightDrive.setPower(Math.abs(speed));

        // Wait until done or timed out
        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (FrontLeftDrive.isBusy() && FrontRightDrive.isBusy() && BackLeftDrive.isBusy() && BackRightDrive.isBusy())) {
            // Wait for the movement to finish.
        }

        // Stop all motion
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);

        // Reset modes for next command
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    }
    public boolean isDriving() { return false; }

}
