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
import com.qualcomm.robotcore.hardware.Servo;

/**
 * @brief wrapper around the `Servo` class to add encoder feedback
 */
public class Axon {
  private Servo servo; // the servo being controlled
  private AnalogInput encoder; // the encoder of the servo being controlled
  private double toleranceDegrees;

  /**
   * @brief creates an object of the `EncoderServo` class with default parameters
   * @param servo the servo to control
   * @param encoder the encoder of the servo being controlled
   */
  public Axon(Servo servo, AnalogInput encoder) {
    this(servo, encoder, 2.5);
  }

  /**
   * @brief creates an object of the `EncoderServo` class
   * @param servo the servo to control
   * @param encoder the encoder of the servo being controlled
   * @param toleranceDegrees the amount the angle read can differ from the target angle and the
   *     servo still be considered "at target"
   */
  public Axon(Servo servo, AnalogInput encoder, double toleranceDegrees) {
    this.servo = servo;
    this.encoder = encoder;
    this.toleranceDegrees = toleranceDegrees;
  }

  /**
   * @brief gets the amount the angle read can differ from the target angle and the servo still be
   *     considered "at target"
   * @return the tolerance, in degrees
   */
  public double getToleranceDegrees() {
    return toleranceDegrees;
  }

  /**
   * @brief sets the amount the angle read can differ from the target angle and the servo still be *
   *     considered "at target"
   * @param toleranceDegrees the tolerance, in degrees
   */
  public void setToleranceDegrees(double toleranceDegrees) {
    this.toleranceDegrees = toleranceDegrees;
  }

  /**
   * @brief sets the target position of the servo
   * @param degrees
   */
  public void setPosition(double degrees) {
    servo.setPosition(degrees / 360);
  }

  /**
   * @brief gets the target position of the servo
   * @return the target position of the servo
   * @note this method doesn't return the *current position*, it returns the *target position*
   */
  public double getTargetPosition() {
    return servo.getPosition() * 360;
  }

  /**
   * @brief reads the current position of the servo
   * @return the current position of the servo, in degrees (from 0 to 360)
   */
  public double getPosition() {
    return (encoder.getVoltage() / 3.3) * 360; // 0v = 0 degrees, 3.3v = 360 degrees
  }

  /**
   * @brief gets if the servo is currently within tolerance of its target
   * @return true if the servo is at its target, false if it isn't at its target
   */
  public boolean isAtTarget() {
    double target = getTargetPosition();
    double position = getPosition();
    return position >= (target - toleranceDegrees) && position <= (target + toleranceDegrees);
  }
}
