package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Autonomous(name="Drive", group="Linear OpMode")
public class AutonDrive extends LinearOpMode {
    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    double drive = -1;
    double strafe = 0;
    double turn  =  0;
    private final ElapsedTime timer = new ElapsedTime();
    public int leftEncoderTicks = leftDrive.getCurrentPosition();
    public int rightEncoderTicks = rightDrive.getCurrentPosition();
    public int frontLeftEncoderTicks = frontLeftDrive.getCurrentPosition();
   public  int frontRightEncoderTicks = frontRightDrive.getCurrentPosition();
    public double leftPower;
    public double rightPower;
    public double strafeLeftPower;
    public double strafeRightPower;
     public void resetRotations() {
        leftEncoderTicks = 0;
        rightEncoderTicks = 0;
        frontLeftEncoderTicks = 0;
        frontRightEncoderTicks = 0;
    }

    public void drive(double speed) {

    }
    final double COUNTS_PER_MOTOR_REV = 28; // Example for HD Hex Motor, adjust for your motor

    double leftRotations = (double) leftEncoderTicks / COUNTS_PER_MOTOR_REV;
    double rightRotations = (double) rightEncoderTicks / COUNTS_PER_MOTOR_REV;
    double frontLeftRotations = (double) frontLeftEncoderTicks / COUNTS_PER_MOTOR_REV;
    double frontRightRotations = (double) frontRightEncoderTicks / COUNTS_PER_MOTOR_REV;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);










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

            leftPower    = Range.clip(drive + turn, -0.4, 0.4) ;
            rightPower   = Range.clip(drive - turn, -0.4, 0.4) ;
            strafeLeftPower = Range.clip(strafe, -0.4, 0.4) ;
            strafeRightPower = Range.clip(strafe, -0.4, 0.4) ;
            final double COUNTS_PER_MOTOR_REV = 28; // Example for HD Hex Motor, adjust for your motor
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            driveForRotations(3);


            // Show the elapsed game timer and wheel power.
            telemetry.addData("Status", "Run Time: " );
            telemetry.addData("Motors", "left (%.2f)", leftPower);
            telemetry.update();
        }
    }
    public void driveForRotations(double rotations) {
         resetRotations();
        leftDrive.setPower(leftPower-strafeLeftPower);
        rightDrive.setPower(rightPower+strafeRightPower);
        frontLeftDrive.setPower(leftPower+strafeLeftPower);
        frontRightDrive.setPower(rightPower-strafeRightPower);
        do {
            drive = 0;
            resetRotations();
        } while (leftRotations <= rotations && rightRotations <= rotations && frontLeftRotations <= rotations && frontRightRotations <= rotations);
    }
    public void turnForRotation(double rotation, double direction) throws InterruptedException {
        if(direction<0){
            resetRotations();
        leftDrive.setPower(-leftPower-strafeLeftPower);
        rightDrive.setPower(rightPower+strafeRightPower);
        frontLeftDrive.setPower(-leftPower+strafeLeftPower);
        frontRightDrive.setPower(rightPower-strafeRightPower);

    }
        else {
            leftDrive.setPower(leftPower-strafeLeftPower);
            rightDrive.setPower(-rightPower+strafeRightPower);
            frontLeftDrive.setPower(leftPower+strafeLeftPower);
            frontRightDrive.setPower(-rightPower-strafeRightPower);

        }
        do {
            drive = 0;
            resetRotations();
        } while (leftRotations <= rotation && rightRotations <= rotation && frontLeftRotations <= rotation && frontRightRotations <= rotation);
        }
        public void strafeForRotation() {


        }

    }

