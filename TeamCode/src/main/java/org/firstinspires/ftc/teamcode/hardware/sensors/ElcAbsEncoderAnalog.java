package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.TestOnly;

/**
 * Analog-only driver for the ELC Encoder V2.
 *
 * - Uses ONLY the analog absolute output
 * - Software unwraps angle to allow multi-turn tracking
 * - Supports inversion and offsets
 */
public class ElcAbsEncoderAnalog {

  private final AnalogInput absoluteEncoder;

  // Datasheet constants
  private static final double MAX_VOLTAGE = 3.3;

  // State
  private double angleOffsetDegrees = 0.0;
  private boolean inverted = false;

  private double lastAbsoluteAngle = 0.0;
  private int revolutionCount = 0;
  private boolean firstRead = true;

  /**
   * Constructor
   *
   * @param hardwareMap OpMode hardware map
   * @param analogName  Name of the Analog Input device
   */
  public ElcAbsEncoderAnalog(HardwareMap hardwareMap, String analogName) {
    this.absoluteEncoder = hardwareMap.get(AnalogInput.class, analogName);
    reset();
  }

  /**
   * Resets multi-turn tracking and zeroes the encoder.
   */
  public void reset() {
    firstRead = true;
    revolutionCount = 0;
    angleOffsetDegrees = 0.0;
    lastAbsoluteAngle = getAbsoluteAngle();
  }

  /**
   * Returns raw absolute angle (0–360)
   */
  private double getAbsoluteAngle() {
    double voltage = absoluteEncoder.getVoltage();
    voltage = Range.clip(voltage, 0.0, absoluteEncoder.getMaxVoltage());

    double angle = (voltage / absoluteEncoder.getMaxVoltage()) * 360.0;

    if (inverted) {
      angle = 360.0 - angle;
    }

    return angle;
  }

  /**
   * Updates revolution tracking.
   * Call this at least once per loop.
   */
  private void update() {
    double currentAngle = getAbsoluteAngle();

    if (firstRead) {
      lastAbsoluteAngle = currentAngle;
      firstRead = false;
      return;
    }

    double delta = currentAngle - lastAbsoluteAngle;

    // Detect wraparound
    if (delta > 180.0) {
      revolutionCount--;
    } else if (delta < -180.0) {
      revolutionCount++;
    }

    lastAbsoluteAngle = currentAngle;
  }

  /**
   * Returns continuous position in degrees (multi-turn).
   */
  public double getPosition() {
    update();
    return (revolutionCount * 360.0) + lastAbsoluteAngle + angleOffsetDegrees;
  }

  /**
   * Returns position normalized to [-180, 180)
   */
  public double getPositionNormalized() {
    return AngleUnit.normalizeDegrees(getPosition());
  }

  /**
   * Sets the current position as zero.
   */
  public void zero() {
    update();
    angleOffsetDegrees = -(revolutionCount * 360.0 + lastAbsoluteAngle);
  }

  /**
   * Inverts encoder direction.
   */
  public void setInverted(boolean inverted) {
    this.inverted = inverted;
    reset();
  }

  /**
   * Debug helper
   */
  @TestOnly
  public double getRawVoltage() {
    return absoluteEncoder.getVoltage();
  }
}
