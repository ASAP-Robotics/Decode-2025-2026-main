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

package org.firstinspires.ftc.teamcode.hardware.servos;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.hardware.thirdparty.RTPAxon;

// Written mostly by Gemini

/**
 * An extension of RTPAxon that enforces a unidirectional path for the setpoint. If given a position
 * that would require reversing, this class automatically "rolls over" the target
 * (adding/subtracting 360 degrees) so the servo reaches the correct angle by continuing in the
 * allowed direction.
 */
public class UnidirectionalAxon extends RTPAxon {
  public enum DirectionConstraint {
    NONE, // Standard shortest-path behavior
    FORWARD_ONLY, // Target must always be > current position (Clockwise/Increasing)
    REVERSE_ONLY // Target must always be < current position (Counter-Clockwise/Decreasing)
  }

  private DirectionConstraint directionConstraint = DirectionConstraint.FORWARD_ONLY;

  // region Constructors

  public UnidirectionalAxon(CRServo servo, AnalogInput encoder) {
    super(servo, encoder);
  }

  public UnidirectionalAxon(CRServo servo, AnalogInput encoder, Direction direction) {
    super(servo, encoder, direction);
  }

  // endregion

  /**
   * Sets the direction constraint.
   *
   * @param constraint The direction the servo is allowed to travel to reach new setpoints.
   */
  public void setDirectionConstraint(DirectionConstraint constraint) {
    this.directionConstraint = constraint;
  }

  public DirectionConstraint getDirectionConstraint() {
    return directionConstraint;
  }

  /**
   * Sets the target rotation. If a constraint is active, this method adjusts the input 'target' by
   * adding or subtracting multiples of 360 degrees to ensure the new target lies in the allowed
   * direction relative to the current total rotation.
   *
   * @param target The desired target angle (absolute degrees, or 0-360).
   */
  @Override
  public void setTargetRotation(double target) {
    double currentPos = isAtTarget() ? getTargetRotation() : getTotalRotation();
    double adjustedTarget = target;
    final double ROTATION_DEG = 360.0;

    switch (directionConstraint) {
      case FORWARD_ONLY:
        // Add rotations until the target is ahead of the current position
        while (adjustedTarget < currentPos) {
          adjustedTarget += ROTATION_DEG;
        }
        break;

      case REVERSE_ONLY:
        // Subtract rotations until the target is behind the current position
        while (adjustedTarget > currentPos) {
          adjustedTarget -= ROTATION_DEG;
        }
        break;

      case NONE:
      default:
        // Pass through unchanged
        break;
    }

    super.setTargetRotation(adjustedTarget);
  }
}
