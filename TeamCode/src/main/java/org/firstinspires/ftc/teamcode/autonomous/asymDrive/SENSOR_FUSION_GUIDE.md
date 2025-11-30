# Multi-Sensor Fusion Guide

## Overview

The FusedLocalizer combines multiple sensors for robust localization:

1. **OTOS (Primary)**: Fast, accurate optical tracking
2. **Drive Encoders (Backup)**: Reliable when OTOS fails
3. **AprilTags (Corrections)**: Absolute position resets

## How It Works

### Complementary Filter
- **OTOS**: High-frequency tracking (good for fast movements)
- **Encoders**: Low-frequency backup (resistant to slip)
- **Blend**: Dynamically weighted based on reliability

### Outlier Detection
The system detects when OTOS gives bad readings:
- **Position jumps** > 10 inches
- **Velocity jumps** > 100 in/s
- Automatically reduces OTOS trust and increases encoder trust

### Reliability Tracking
- Starts at 100% trust in OTOS
- Decays to 10% minimum when outliers detected
- Gradually recovers when readings stabilize

## Setup Instructions

### 1. Tune Drive Encoder Constants

In `ConstantsFused.java`, update these values:

```java
.setForwardTicksToInches(1.0 / TICKS_PER_REV)
.setLateralTicksToInches(1.0 / TICKS_PER_REV)
```

**To find TICKS_PER_REV:**
- GoBILDA 312 RPM (5203): 537.7 ticks/rev
- GoBILDA 435 RPM (5202): 383.6 ticks/rev
- AndyMark NeveRest 40: 1120 ticks/rev
- REV HD Hex 40:1: 1120 ticks/rev

### 2. Calibrate Encoder Multipliers

Run the Pedro Pathing tuning OpModes:
- **Forward Tuner**: Calibrate forward movement
- **Lateral Tuner**: Calibrate strafing
- Update `setXMultiplier()` and `setYMultiplier()` with the results

### 3. Adjust Fusion Weights

Default: 70% OTOS, 30% Encoders

```java
fusedLocalizer.setFusionWeights(0.7);  // 70% OTOS
```

**When to adjust:**
- **Slippery floor**: Reduce to 0.5 (50% OTOS, 50% encoders)
- **Perfect floor**: Increase to 0.9 (90% OTOS, 10% encoders)
- **OTOS fails often**: Reduce to 0.3 (30% OTOS, 70% encoders)

### 4. Enable AprilTag Correction (Optional)

If you have a camera, add AprilTag absolute position corrections:

```java
// In your OpMode init()
AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
    .setDrawAxes(true)
    .setDrawCubeProjection(true)
    .setDrawTagOutline(true)
    .build();

VisionPortal visionPortal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    .addProcessor(aprilTagProcessor)
    .build();

// Enable AprilTag correction in FusedLocalizer
fusedLocalizer.enableAprilTagCorrection(aprilTagProcessor);
```

**TODO**: Implement `calculateRobotPoseFromTag()` in FusedLocalizer.java
- You need to know AprilTag field positions
- You need camera-to-robot transformation
- You need to transform from tag frame to field frame

## Testing Procedure

### Test 1: OTOS Health Check
1. Run `Fused Localization Test`
2. Drive the robot around smoothly
3. Watch "OTOS Reliability" - should stay > 80%
4. If it drops frequently, your floor surface is problematic

### Test 2: Slip Detection
1. Place robot on field
2. Drive forward 48 inches at full speed
3. Slam on the brakes (let wheels slip)
4. Watch reliability drop as it detects the slip
5. Encoder contribution should increase

### Test 3: Different Surfaces
Test on different floor materials:
- **Smooth tiles**: OTOS should work perfectly (90%+ reliability)
- **Foam tiles**: OTOS might struggle (60-80% reliability)
- **Carpet**: OTOS will fail (30-50% reliability, encoders take over)

### Test 4: Autonomous Consistency
1. Create a simple square path
2. Run it 10 times
3. Measure ending position variance
4. **Goal**: < 2 inch variance

## Troubleshooting

### OTOS Reliability Always Low
**Symptom**: Reliability drops to 30-40% immediately

**Causes:**
1. Floor surface incompatible with OTOS
2. OTOS mounted too high or at wrong angle
3. OTOS lens dirty or scratched

**Solutions:**
- Clean OTOS lens
- Check mounting height (should be 1-2 inches from floor)
- Adjust fusion weights to rely more on encoders

### Erratic Position Jumps
**Symptom**: Position jumps around randomly

**Causes:**
1. Both OTOS and encoders giving bad data
2. Encoder constants not calibrated
3. Wheel slip on both sides simultaneously

**Solutions:**
- Calibrate encoder multipliers
- Add dedicated odometry wheels (not using drive motors)
- Increase encoder weight in fusion

### Autonomous Drift
**Symptom**: Robot drifts off path over time

**Causes:**
1. Encoder calibration drift
2. OTOS surface sensitivity
3. No absolute position corrections

**Solutions:**
- Implement AprilTag corrections
- Recalibrate encoders
- Use dedicated odometry wheels

## Advanced: Adding Dedicated Odometry Wheels

For best results, replace drive encoders with dedicated odometry wheels:

**Benefits:**
- No wheel slip (omni wheels don't slip)
- More accurate than drive encoders
- Independent of drive motor behavior

**Implementation:**
1. Mount 3 odometry wheels (2 parallel, 1 perpendicular)
2. Use Pedro Pathing's `ThreeWheelLocalizer`
3. Replace `DriveEncoderLocalizer` with `ThreeWheelLocalizer` in `ConstantsFused.java`

## Performance Metrics

**Good Fusion Performance:**
- OTOS Reliability: 80-95%
- Encoder Contribution: 5-20%
- Position Drift: < 2 inches per 100 inches traveled
- Autonomous Repeatability: ± 1 inch

**Acceptable Performance:**
- OTOS Reliability: 60-80%
- Encoder Contribution: 20-40%
- Position Drift: < 4 inches per 100 inches traveled
- Autonomous Repeatability: ± 2 inches

**Poor Performance (needs adjustment):**
- OTOS Reliability: < 60%
- Position Drift: > 4 inches per 100 inches traveled
- Autonomous Repeatability: > 3 inches

## Future Enhancements

### 1. Kalman Filter
Replace complementary filter with Kalman filter for optimal fusion:
- Better handling of sensor noise
- Optimal weighting based on sensor covariance
- Predictive capability

### 2. AprilTag Implementation
Complete the AprilTag correction system:
- Field-relative AprilTag positions
- Camera extrinsics (camera to robot transform)
- Pose estimation from tag detections

### 3. IMU Integration
Add gyroscope for better heading estimation:
- Fuse OTOS heading, encoder heading, and IMU heading
- Better handling of turns and spins

### 4. Machine Learning
Train a model to predict sensor reliability:
- Floor surface classification
- Predictive outlier detection
- Adaptive fusion weights

## References

- Pedro Pathing Documentation: https://pedropathing.com
- FTC AprilTag Guide: https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
- Sensor Fusion Theory: https://en.wikipedia.org/wiki/Sensor_fusion
