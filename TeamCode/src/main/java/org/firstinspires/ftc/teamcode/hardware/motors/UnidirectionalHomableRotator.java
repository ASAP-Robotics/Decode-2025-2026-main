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
import org.firstinspires.ftc.teamcode.hardware.sensors.BreakBeam;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

/**
 * An extension of HomableRotator that allows enforcing direction constraints on movement.
 * @note Does not support multi-turn commands.
 */
public class UnidirectionalHomableRotator extends HomableRotator {
  public enum DirectionConstraint {
    NONE, // Standard shortest-path behavior
    FORWARD_ONLY, // Target must always be > current position (Increasing)
    REVERSE_ONLY // Target must always be < current position (Decreasing)
  }

  protected DirectionConstraint directionConstraint = DirectionConstraint.NONE;

  public UnidirectionalHomableRotator(Motor motor, BreakBeam sensor, double kp, double ki, double kd, double tolerance, boolean inverted) {
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
    double currentAngle = getCurrentAngle();

    switch (directionConstraint) {
      case FORWARD_ONLY:
        while (degrees - 360 >= currentAngle) degrees -= 360; // remove unneeded turns
        while (degrees < currentAngle) degrees += 360; // ensure direction
        break;
      case REVERSE_ONLY:
        while (degrees + 360 <= currentAngle) degrees += 360; // remove unneeded turns
        while (degrees > currentAngle) degrees -= 360; // ensure direction
        break;
      case NONE:
      default:
        // Standard shortest-path behavior
        degrees = MathUtils.closestWithOffset(degrees, currentAngle, 360);
        break;
    }

    super.setTargetAngle(degrees);
  }
}
