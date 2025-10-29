/*
 * Copyright 2025 ASAP Robotics (FTC Team 22029)
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

/**
 * @brief a class that makes two continuous rotation servo behave like a single normal servo that
 *     only moves in one direction to achieve target positions
 * @note this is just a wrapper around the RTPAxon class
 */
public class MonodirectionalDualServo {
  private final RTPAxon servo1, servo2; // class to handle most servo control
  // acceptable position error in degrees (if position is within <this> of the target, reverse
  // movement to maintain position is OK)
  private double toleranceDegrees;
  private double targetPositionDegrees; // target position of the servo in degrees

  /**
   * @brief makes a new MonodirectionalDualServo
   * @param servo1 the first servo to control
   * @param servo2 the second servo to control
   * @param encoder the encoder to read to get the servo's positions (one of the servo's encoders)
   * @param toleranceDegrees the position error in degrees where the servo is still "at target"
   * @param direction the direction of the servo
   */
  public MonodirectionalDualServo(
      CRServo servo1,
      CRServo servo2,
      AnalogInput encoder,
      double toleranceDegrees,
      RTPAxon.Direction direction) {
    this.toleranceDegrees = toleranceDegrees;
    this.servo1 = new RTPAxon(servo1, encoder, direction);
    this.servo2 = new RTPAxon(servo2, encoder, direction);
    // start with target at current position
    this.targetPositionDegrees = this.servo1.getCurrentAngle();
  }

  /**
   * @brief makes a new MonodirectionalDualServo with default parameters
   * @param servo1 the servo to control
   */
  public MonodirectionalDualServo(CRServo servo1, CRServo servo2, AnalogInput encoder) {
    this(servo1, servo2, encoder, 1.0, RTPAxon.Direction.FORWARD);
  }

  /**
   * @brief sets the threshold of error (in degrees) above which the servo is no longer at the
   *     target position
   * @param toleranceDegrees the number of degrees the position can differ from the target position
   */
  public void setToleranceDegrees(double toleranceDegrees) {
    this.toleranceDegrees = toleranceDegrees;
  }

  /**
   * @brief sets the target position of the servo (like servo.setPosition())
   * @param position the target position of the servo, in degrees (0 to 360)
   * @note does not update the servo's movement; call update() to update movement
   */
  public void setPosition(double position) {
    // filter target to be between 0 and 360
    double filteredPosition = Math.min(Math.abs(position), 360.0);

    if (targetPositionDegrees - toleranceDegrees > filteredPosition
        || targetPositionDegrees + toleranceDegrees < filteredPosition) {
      // ^ if new target is not the same as old target
      double change = 360 - (targetPositionDegrees - filteredPosition); // get change
      while (change >= 360) change -= 360; // change should never be more than 360

      servo1.changeTargetRotation(change);
      servo2.changeTargetRotation(change);

    } else { // new target is "the same" as old target
      servo1.setTargetRotation(filteredPosition);
      servo2.setTargetRotation(filteredPosition);
    }

    targetPositionDegrees = filteredPosition; // store new target
  }

  /**
   * @brief gets if the servo(s) are at the target position
   * @return true if the servo(s) are at the target position, false otherwise
   */
  public boolean isAtTarget() {
    return servo1.isAtTarget(toleranceDegrees) && servo2.isAtTarget(toleranceDegrees);
  }

  /**
   * @brief gets the servo's target position in degrees
   * @return the servo's target position, in degrees
   */
  public double getTargetPosition() {
    return targetPositionDegrees;
  }

  /**
   * @brief updates the servo's movement. Call every loop
   */
  public void update() {
    servo1.update();
    servo2.update();
  }
}
