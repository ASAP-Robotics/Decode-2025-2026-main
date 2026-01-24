/*
 * Copyright 2026 ASAP Robotics (FTC Team 22029)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Unified driver for the ELC Encoder V2.
 * * Functionality:
 * - Uses the Analog connection to "seed" the initial position (Absolute).
 * - Uses the Quadrature connection for runtime tracking (Incremental).
 * - Supports software offsets and direction inversion.
 * * Datasheet Specs:
 * - Analog Output: 0-3.3V [cite: 6]
 * - Quadrature Resolution: 4000 CPR [cite: 6]
 *
 * @note written by Gemini, still needs to be tested
 */
public class ElcAbsEncoder {
  private final AnalogInput absoluteEncoder;
  private final DcMotorEx incrementalEncoder;

  // Datasheet Constants
  private static final double MAX_VOLTAGE = 3.3; // [cite: 6]
  private static final double TICKS_PER_REV = 4000.0; // [cite: 6]

  // State variables
  private double angleOffsetDegrees = 0.0;
  private boolean inverted = false;

  // Configurable noise reduction
  private int syncSampleCount = 5;

  /**
   * Constructor.
   * Automatically attempts to synchronize position upon instantiation.
   * * @param hardwareMap   The OpMode's hardware map.
   * @param analogName    Name of the device in Config -> Analog Input.
   * @param digitalName   Name of the device in Config -> Motors.
   */
  public ElcAbsEncoder(HardwareMap hardwareMap, String analogName, String digitalName) {
    // Initialize Hardware
    this.absoluteEncoder = hardwareMap.get(AnalogInput.class, analogName);
    this.incrementalEncoder = hardwareMap.get(DcMotorEx.class, digitalName);

    // Reset the incremental encoder hardware to ensure a clean slate
    // Note: We do NOT use RUN_USING_ENCODER as that engages PID control.
    // We just want raw data reading.
    this.incrementalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.incrementalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // Perform initial synchronization
    synchronize();
  }

  /**
   * Reads the Absolute (Analog) encoder and forces the internal offset
   * to match the incremental encoder to this real-world angle.
   * * Uses a multi-read average to reject noise.
   */
  public void synchronize() {
    double absoluteAngle = getAbsoluteAngle();

    // Get current incremental position (usually 0 if we just reset, but good for safety)
    double currentIncrementalTicks = incrementalEncoder.getCurrentPosition();
    double currentIncrementalDegrees = (currentIncrementalTicks / TICKS_PER_REV) * 360.0;

    // Calculate the offset required to make (Incremental + Offset) = Absolute
    // Formula: Offset = Target - Current
    this.angleOffsetDegrees = absoluteAngle - currentIncrementalDegrees;
  }

  /**
   * Gets the absolute angle
   * @return the current absolute angle of the encoder
   * @note auto-generated
   */
  private double getAbsoluteAngle() {
    double avgVoltage = 0;

    // Block briefly to average samples for accuracy
    for (int i = 0; i < syncSampleCount; i++) {
      avgVoltage += absoluteEncoder.getVoltage();
    }
    avgVoltage /= syncSampleCount;

    // Clip voltage to expected range [cite: 6]
    avgVoltage = Range.clip(avgVoltage, 0, MAX_VOLTAGE);

    // Calculate Absolute Angle (0-360)
    double absoluteAngle = (avgVoltage / MAX_VOLTAGE) * 360.0;

    // If the mechanism is inverted, flip the absolute read logic
    if (inverted) {
      absoluteAngle = 360.0 - absoluteAngle;
    }
    return absoluteAngle;
  }

  /**
   * Gets the current hybrid position in degrees.
   * @return Position in degrees (0-360, or multi-turn if tracked continuously).
   */
  public double getPosition() {
    // 1. Get Incremental Phase
    double currentTicks = incrementalEncoder.getCurrentPosition();
    if (inverted) {
      currentTicks = -currentTicks;
    }

    // 2. Convert to Degrees
    double incrementalDegrees = (currentTicks / TICKS_PER_REV) * 360.0;

    // 3. Apply the Synchronized Offset
    return incrementalDegrees + angleOffsetDegrees;
  }

  /**
   * Gets the current position normalized to 0-360 degrees.
   * Useful for swerve modules where 361 degrees should be 1 degree.
   */
  public double getPositionNormalized() {
    double angle = getPosition() % 360.0;
    if (angle < 0) {
      angle += 360.0;
    }
    return angle;
  }

  /**
   * Gets velocity in Degrees per Second.
   * Uses the high-speed Quadrature signal.
   */
  public double getVelocity() {
    double velTicks = incrementalEncoder.getVelocity();
    if (inverted) {
      velTicks = -velTicks;
    }
    return (velTicks / TICKS_PER_REV) * 360.0;
  }

  /**
   * Sets the direction of the encoder.
   * If true, inverts both the incremental count and the absolute logic.
   * * @param inverted true to invert.
   */
  public void setInverted(boolean inverted) {
    this.inverted = inverted;
    // Re-sync immediately because inversion changes the absolute target
    synchronize();
  }

  /**
   * Configure how many samples to average during a synchronize() call.
   * Default is 5. Higher numbers reduce noise but take slightly longer.
   */
  public void setSyncSampleCount(int count) {
    this.syncSampleCount = count;
  }

  /**
   * Returns the raw voltage from the absolute line (Debug).
   */
  public double getRawVoltage() {
    return absoluteEncoder.getVoltage();
  }
}