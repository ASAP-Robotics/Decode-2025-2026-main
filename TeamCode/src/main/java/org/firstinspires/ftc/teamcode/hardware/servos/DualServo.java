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

import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;

/**
 * @brief simple class to control two Axon servos at once
 */
public class DualServo implements System {
  private final Axon servo1, servo2; // servos
  private double targetPosition; // the angle to move the servos to

  public DualServo(Axon servo1, Axon servo2) {
    this.servo1 = servo1;
    this.servo2 = servo2;
  }

  public SystemReport getStatus() {
    SystemStatus status = SystemStatus.NOMINAL;
    SystemStatus s1Status = servo1.getStatus().status;
    SystemStatus s2Status = servo1.getStatus().status;
    String message = "Operational";

    if (s1Status == SystemStatus.FALLBACK || s2Status == SystemStatus.FALLBACK) {
      status = SystemStatus.FALLBACK;
      message = s1Status == SystemStatus.FALLBACK ? "Servo 1 in fallback; performance will be degraded"
          : "Servo 2 in fallback; performance will be degraded";
    }

    if (s1Status == SystemStatus.INOPERABLE || s2Status == SystemStatus.INOPERABLE) {
      status = SystemStatus.INOPERABLE;
      message = s1Status == SystemStatus.INOPERABLE ? "Servo 1 inoperable"
          : "Servo 2 inoperable";
    }

    return new SystemReport(status, message);
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
