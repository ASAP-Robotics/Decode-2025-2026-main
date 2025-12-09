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

import static java.lang.Math.max;
import static java.lang.Math.min;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.map;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Follower;

/**
 * @brief wrapper around the `Servo` class to add encoder feedback
 */
public class Axon {
  private final Follower follower; // backup follower to model servo movement if encoder fails
  private final Servo servo; // the servo being controlled
  private final AnalogInput encoder; // the encoder of the servo being controlled
  private final boolean dummy; // if true, the servo will always be "at target"
  private final double toleranceDegrees;

  /**
   * @brief creates a dummy (no encoder) Axon with default parameters
   * @param servo the servo to control
   */
  public Axon(Servo servo) {
    this(servo, null, 5, true);
  }

  /**
   * @brief creates an object of the `EncoderServo` class with default parameters
   * @param servo the servo to control
   * @param encoder the encoder of the servo being controlled
   */
  public Axon(Servo servo, AnalogInput encoder) {
    this(servo, encoder, 5, false);
  }

  /**
   * @brief creates an object of the `EncoderServo` class
   * @param servo the servo to control
   * @param encoder the encoder of the servo being controlled
   * @param toleranceDegrees the amount the angle read can differ from the target angle and the
   *     servo still be considered "at target"
   */
  public Axon(Servo servo, AnalogInput encoder, double toleranceDegrees, boolean dummy) {
    this.servo = servo;
    this.encoder = encoder;
    this.dummy = dummy;
    this.toleranceDegrees = toleranceDegrees;
    // 214 degrees per second, about the speed of an axon divided by 2
    this.follower = dummy ? null : new Follower(getPosition(), 0, 0, 214);
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
   * @brief sets the target position of the servo
   * @param degrees the target position of the servo, in degrees
   */
  public void setPosition(double degrees) {
    servo.setPosition(degrees / 360);
  }

  /**
   * @brief gets the target position of the servo
   * @return the target position of the servo, or 0 if it hasn't been set yet
   * @note this method doesn't return the *current position*, it returns the *target position*
   */
  public double getTargetPosition() {
    double target = servo.getPosition() * 360;
    return Double.isNaN(target) ? target : 0;
  }

  /**
   * @brief reads the current position of the servo
   * @return the current position of the servo, in degrees (from 0 to 360)
   * @note if this servo is a dummy this will always return 0
   */
  public double getPosition() {
    if (dummy) return 0; // if dummy we can't get position
    // 0v = 0 degrees, 3.3v = 360 degrees
    return min(360, max(0, map(360 - ((encoder.getVoltage() / 3.3) * 360), 20, 340, 0, 360)));
  }

  /**
   * @brief gets if the servo is currently within tolerance of its target
   * @return true if the servo is at its target, false if it isn't at its target
   * @note if this servo is a dummy this will always return true
   */
  public boolean isAtTarget() {
    if (dummy) return true; // dummy servos are always at target
    boolean encoderAtTarget = Math.abs(getTargetPosition() - getPosition()) <= toleranceDegrees;
    boolean followerAtTarget = follower.isAtTarget();
    return encoderAtTarget || followerAtTarget;
  }
}
