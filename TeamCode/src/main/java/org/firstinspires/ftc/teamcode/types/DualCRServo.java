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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

/** A class that combines two CRServos into one logical CRServo, controlling both simultaneously. */
public class DualCRServo implements CRServo {
  private final CRServo servo0; // "master" servo
  private final CRServo servo1;

  /**
   * Makes a single CRServo that controls two CRServos
   *
   * @param servo0 first (primary) servo
   * @param servo1 second (secondary) servo
   */
  public DualCRServo(CRServo servo0, CRServo servo1) {
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
  public void setPower(double power) {
    servo0.setPower(power);
    servo1.setPower(power);
  }

  @Override
  public double getPower() {
    return servo0.getPower();
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
