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

package org.firstinspires.ftc.teamcode.types;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/** A class that combines two Servos into one logical Servo, controlling both simultaneously. */
public class DualServo implements Servo {
  private final Servo servo0; // "master" servo
  private final Servo servo1;

  /**
   * Makes a single Servo that controls two Servos
   *
   * @param servo0 first (primary) servo
   * @param servo1 second (secondary) servo
   */
  public DualServo(Servo servo0, Servo servo1) {
    this.servo0 = servo0;
    this.servo1 = servo1;
  }

  @Override
  public ServoController getController() {
    return servo0.getController();
  }

  @Override
  public int getPortNumber() {
    return servo0.getPortNumber();
  }

  @Override
  public void setDirection(Direction direction) {
    servo0.setDirection(direction);
    servo1.setDirection(direction);
  }

  @Override
  public Direction getDirection() {
    return servo0.getDirection();
  }

  @Override
  public void setPosition(double position) {
    servo0.setPosition(position);
    servo1.setPosition(position);
  }

  @Override
  public double getPosition() {
    return servo0.getPosition();
  }

  @Override
  public void scaleRange(double min, double max) {
    servo0.scaleRange(min, max);
    servo1.scaleRange(min, max);
  }

  @Override
  public Manufacturer getManufacturer() {
    return servo0.getManufacturer();
  }

  @Override
  public String getDeviceName() {
    return servo0.getDeviceName();
  }

  @Override
  public String getConnectionInfo() {
    return servo0.getConnectionInfo();
  }

  @Override
  public int getVersion() {
    return servo0.getVersion();
  }

  @Override
  public void resetDeviceConfigurationForOpMode() {
    servo0.resetDeviceConfigurationForOpMode();
    servo1.resetDeviceConfigurationForOpMode();
  }

  @Override
  public void close() {
    servo0.close();
    servo1.close();
  }
}
