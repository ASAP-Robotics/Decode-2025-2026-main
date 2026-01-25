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

package org.firstinspires.ftc.teamcode.hardware.motors;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

/**
 * An extension of HomableRotator that allows enforcing direction constraints on movement.
 *
 * @note Does not support multi-turn commands (moving more than 360 degrees from current position).
 *     It should be noted that moving up to exactly 360 degrees from current position *is*
 *     supported.
 */
public class UnidirectionalHomableRotator extends HomableRotator {
  public enum DirectionConstraint {
    NONE, // Standard behavior
    FORWARD_ONLY, // Target will be adjusted to always be > current position (Increasing)
    REVERSE_ONLY // Target will be adjusted to always be < current position (Decreasing)
  }

  protected DirectionConstraint directionConstraint = DirectionConstraint.NONE;

  public UnidirectionalHomableRotator(
      Motor motor,
      TouchSensor sensor,
      double kp,
      double ki,
      double kd,
      double tolerance,
      boolean inverted) {
    super(motor, sensor, kp, ki, kd, tolerance, inverted);
  }

  /**
   * Sets the direction constraint.
   *
   * @param constraint The direction the motor is allowed to travel to reach new setpoints.
   */
  public void setDirectionConstraint(DirectionConstraint constraint) {
    directionConstraint = constraint;
  }

  /**
   * Gets the current direction constraint.
   *
   * @return the current direction constraint.
   */
  public DirectionConstraint getDirectionConstraint() {
    return directionConstraint;
  }

  /**
   * Sets the target angle of the motor
   *
   * @param degrees the new target angle
   */
  @Override
  public void setTargetAngle(double degrees) {
    if (Double.isNaN(degrees) || Double.isInfinite(degrees)) return;
    super.setTargetAngle(calculateSetAngle(degrees, directionConstraint));
  }

  /**
   * Calculates the actual setpoint to be applied from a given target angle
   *
   * @param degrees the angle to convert to a setpoint
   * @param constraint the direction constraint to evaluate the setpoint using
   * @return a setpoint that will move the motor to the given angle under the applied constraints
   */
  private double calculateSetAngle(double degrees, DirectionConstraint constraint) {
    // double currentAngle = atTarget() ? getTargetAngle() : getCurrentAngle();
    double currentAngle = getTargetAngle(); // chat strategy

    switch (constraint) {
      case FORWARD_ONLY:
        while (degrees - 360 > currentAngle) degrees -= 360; // remove unneeded turns
        while (degrees < currentAngle) degrees += 360; // ensure direction
        break;
      case REVERSE_ONLY:
        while (degrees + 360 < currentAngle) degrees += 360; // remove unneeded turns
        while (degrees > currentAngle) degrees -= 360; // ensure direction
        break;
      case NONE:
      default:
        // Standard shortest-path behavior
        degrees = MathUtils.closestWithOffset(degrees, currentAngle, 360);
        break;
    }

    return degrees;
  }

  @Override
  public double getAngleTravel(double angle) {
    if (Double.isNaN(angle) || Double.isInfinite(angle)) return Double.POSITIVE_INFINITY;
    return Math.abs(getCurrentAngle() - calculateSetAngle(angle, directionConstraint));
  }

  /**
   * Gets the distance (in degrees) between the given angle and the current angle. Basically, if the
   * given angle was set as the target, how far would the motor move?
   *
   * @param angle the "target" to measure the distance from
   * @param constraint the direction constraint to simulate under
   * @return the distance from the given target, and the current angle
   * @note returns positive infinity on invalid parameters
   */
  public double getAngleTravel(double angle, DirectionConstraint constraint) {
    if (Double.isNaN(angle) || Double.isInfinite(angle)) return Double.POSITIVE_INFINITY;
    return Math.abs(getCurrentAngle() - calculateSetAngle(angle, constraint));
  }
}
