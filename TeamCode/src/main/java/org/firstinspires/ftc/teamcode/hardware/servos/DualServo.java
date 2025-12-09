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

/**
 * @brief simple class to control two Axon servos at once
 */
public class DualServo {
  private final Axon servo1, servo2; // servos
  private double targetPosition; // the angle to move the servos to

  public DualServo(Axon servo1, Axon servo2) {
    this.servo1 = servo1;
    this.servo2 = servo2;
  }

  /**
   * @param degrees the angle to turn the servo to
   * @brief sets the position of the servos
   */
  public void setPosition(double degrees) {
    targetPosition = degrees; // store target position
    // set servo angles:
    servo1.setPosition(targetPosition);
    servo2.setPosition(targetPosition);
  }

  /**
   * @return true if either servo is at target, false otherwise
   * @brief gets if either servo is at the target angle
   */
  public boolean isAtTarget() {
    return servo1.isAtTarget() || servo2.isAtTarget();
  }

  /**
   * @return the target angle of the servos
   * @brief gets the target position of the servos
   */
  public double getTargetPosition() {
    return targetPosition;
  }
}
