package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Diagnostic OpMode to verify OTOS offset configuration.
 * This checks if the offset is being properly set and read back from the sensor.
 */
@TeleOp(name = "OTOS Diagnostic", group = "Diagnostic")
public class OTOSDiagnostic extends OpMode {
    private SparkFunOTOS otos;

    @Override
    public void init() {
        // Initialize OTOS sensor
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        // Configure units
        otos.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        otos.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);

        // Set the offset from Constants
        SparkFunOTOS.Pose2D offset = Constants.otos.offset;
        telemetry.addData("Setting Offset", "X=%.2f Y=%.2f H=%.2f°",
                          offset.x, offset.y, Math.toDegrees(offset.h));

        otos.setOffset(offset);

        // Calibrate IMU
        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        otos.calibrateImu();

        // Reset position
        otos.resetTracking();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read back the offset to verify it was set
        SparkFunOTOS.Pose2D currentOffset = otos.getOffset();

        // Read current pose
        SparkFunOTOS.Pose2D pose = otos.getPosition();

        // Display offset configuration
        telemetry.addLine("=== OFFSET VERIFICATION ===");
        telemetry.addData("Configured Offset", "X=%.2f Y=%.2f H=%.1f°",
                          Constants.otos.offset.x, Constants.otos.offset.y,
                          Math.toDegrees(Constants.otos.offset.h));
        telemetry.addData("OTOS Reports Offset", "X=%.2f Y=%.2f H=%.1f°",
                          currentOffset.x, currentOffset.y, Math.toDegrees(currentOffset.h));

        telemetry.addLine();

        // Display current pose
        telemetry.addLine("=== CURRENT POSE ===");
        telemetry.addData("Position", "X=%.2f Y=%.2f", pose.x, pose.y);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.h));

        telemetry.addLine();
        telemetry.addLine("Instructions:");
        telemetry.addLine("1. Check if 'OTOS Reports Offset' matches 'Configured Offset'");
        telemetry.addLine("2. If they DON'T match, the OTOS isn't accepting the offset");
        telemetry.addLine("3. Place robot at origin (0,0), then rotate it");
        telemetry.addLine("4. Watch if position stays near (0,0) or moves in a circle");

        telemetry.update();
    }
}
