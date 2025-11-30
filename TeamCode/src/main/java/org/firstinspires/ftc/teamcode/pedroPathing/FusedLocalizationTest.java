package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathingNew.FusedLocalizer;

/**
 * Test OpMode for fused localization.
 * Shows real-time sensor fusion status and reliability metrics.
 */
@TeleOp(name = "Fused Localization Test", group = "Pedro Pathing")
public class FusedLocalizationTest extends OpMode {
    private Follower follower;
    private FusedLocalizer fusedLocalizer;

    @Override
    public void init() {
        // Create follower with fused localization
        follower = ConstantsFused.createFusedFollower(hardwareMap);

        // Get reference to fused localizer for diagnostics
        if (follower.getPoseTracker().getLocalizer() instanceof FusedLocalizer) {
            fusedLocalizer = (FusedLocalizer) follower.getPoseTracker().getLocalizer();
        }

        follower.startTeleopDrive();
        telemetry.addData("Status", "Initialized with Fused Localization");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();

        // Display pose
        telemetry.addLine("=== ROBOT POSE ===");
        telemetry.addData("X", "%.2f in", follower.getPose().getX());
        telemetry.addData("Y", "%.2f in", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addLine();

        // Display velocity
        telemetry.addLine("=== VELOCITY ===");
        telemetry.addData("X Vel", "%.2f in/s", follower.getVelocity().getXComponent());
        telemetry.addData("Y Vel", "%.2f in/s", follower.getVelocity().getYComponent());

        telemetry.addLine();

        // Display fusion status
        if (fusedLocalizer != null) {
            telemetry.addLine("=== SENSOR FUSION ===");
            telemetry.addData("Status", fusedLocalizer.getFusionStatus());
            telemetry.addData("OTOS Reliability", "%.1f%%", fusedLocalizer.getOtosReliability() * 100);

            // Visual indicator of OTOS health
            double reliability = fusedLocalizer.getOtosReliability();
            String healthIndicator;
            if (reliability > 0.8) {
                healthIndicator = "✓ Excellent";
            } else if (reliability > 0.6) {
                healthIndicator = "⚠ Good";
            } else if (reliability > 0.4) {
                healthIndicator = "⚠ Fair - Using encoders";
            } else {
                healthIndicator = "✗ Poor - Mostly encoders";
            }
            telemetry.addData("Health", healthIndicator);
        }

        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Left Stick: Drive/Strafe");
        telemetry.addLine("Right Stick X: Turn");
        telemetry.addLine("Press B to reset position");

        // Reset on B button
        if (gamepad1.b) {
            follower.setPose(new Pose(0,0,0));
            telemetry.addData("Action", "Position Reset!");
        }

        telemetry.update();
    }
}
