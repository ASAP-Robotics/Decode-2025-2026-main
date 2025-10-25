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

package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * @brief a class that makes a continuous rotation servo behave like a normal servo that only moves
 *     in one direction to achieve target positions
 */
public class MonodirectionalServo {
  private final CRServo servo; // the servo being controlled
  private final AnalogInput encoder; // the analog input used to read the servo's position
  private double deadZoneDegrees; // size of dead zone to prevent jitter (in degrees)
  private double
      toleranceDegrees; // acceptable position error in degrees (if position is within <this> of the
  // target, reverse movement to maintain position is OK)
  private double slowDownZoneDegrees; // size of the "slow down" zone (in degrees)
  private double targetPositionDegrees; // target position of the servo in degrees
  private double currentPositionDegrees; // last read position of the servo in degrees

  /**
   * @brief makes a new MonodirectionalServo
   * @param servo the servo to control
   * @param toleranceDegrees the acceptable position error in degrees
   * @param slowDownZoneDegrees the size of the zone (in degrees) in which the servo's speed will be
   *     ramped down before reaching the target
   * @param direction the direction of the servo
   */
  public MonodirectionalServo(
      CRServo servo,
      AnalogInput encoder,
      double deadZoneDegrees,
      double toleranceDegrees,
      double slowDownZoneDegrees,
      CRServo.Direction direction) {
    this.servo = servo;
    this.encoder = encoder;
    this.servo.setDirection(direction);
    this.deadZoneDegrees = deadZoneDegrees;
    this.toleranceDegrees = toleranceDegrees;
    this.slowDownZoneDegrees = slowDownZoneDegrees;
    this.currentPositionDegrees = readPosition();
    this.targetPositionDegrees = currentPositionDegrees; // start with target at current position
  }

  /**
   * @brief makes a new MonodirectionalServo with default parameters
   * @param servo the servo to control
   */
  public MonodirectionalServo(CRServo servo, AnalogInput encoder) {
    this(servo, encoder, 2.5, 5.0, 10.0, CRServo.Direction.FORWARD);
  }

  /**
   * @brief sets the size of the zone on either side of the target position in which the servo will
   *     be set to not move
   * @param deadZoneDegrees the size of the dead zone on either side of the target position, in
   *     degrees
   */
  public void setDeadZoneDegrees(double deadZoneDegrees) {
    this.deadZoneDegrees = deadZoneDegrees;
  }

  /**
   * @brief sets the threshold of error (in degrees) above which the servo is no longer at the
   *     target position (and cannot move in reverse to maintain position)
   * @param toleranceDegrees the number of degrees the position can differ from the target position
   */
  public void setToleranceDegrees(double toleranceDegrees) {
    this.toleranceDegrees = toleranceDegrees;
  }

  /**
   * @brief sets the number of degrees away from the target at which the servo will start slowing
   *     down
   * @param slowDownZoneDegrees the distance from the target to slow down at (degrees)
   */
  public void setSlowDownZoneDegrees(double slowDownZoneDegrees) {
    this.slowDownZoneDegrees = slowDownZoneDegrees;
  }

  /**
   * @brief sets the target position of the servo (like servo.setPosition())
   * @param position the target position of the servo, in degrees (0 to 360)
   * @note does not update the servo's movement; call update() to update movement
   */
  public void setPosition(double position) {
    targetPositionDegrees = Math.min(Math.abs(position), 360.0);
  }

  /**
   * @brief gets if the servo is at the target position
   * @return true if the servo is at the target position, false otherwise
   */
  public boolean isAtTarget() {
    double error = getDifferenceDegrees(targetPositionDegrees, currentPositionDegrees);
    return Math.abs(error) <= toleranceDegrees;
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
    currentPositionDegrees = readPosition();
    double error = getDifferenceDegrees(targetPositionDegrees, currentPositionDegrees);

    if (Math.abs(error)
        <= deadZoneDegrees) { // if we are within the dead zone (exactly at the target)
      setSpeed(0); // stop servo

    } else if ((Math.abs(error) <= toleranceDegrees)
        || (error <= slowDownZoneDegrees && error >= 0)) {
      // if we are within the tolerance zone (close to the target) OR if we aren't at the target,
      // but are close enough to start slowing down
      setSpeed(error / slowDownZoneDegrees); // move servo towards target position

    } else { // if we aren't close to the target
      setSpeed(1); // move forwards at full speed
    }
  }

  /**
   * @brief sets the speed of the servo
   * @param speed the speed of the servo (-1 [fully backwords] to 1 [fully forwards])
   */
  private void setSpeed(double speed) {
    speed = Math.max(-1.0, Math.min(1.0, speed)); // constrain speed to -1 to 1
    servo.setPower(speed); // set speed of the servo
  }

  /**
   * @brief reads the current position of the servo
   * @return the current position of the servo, in degrees (from 0 to 360)
   */
  private double readPosition() {
    return (encoder.getVoltage() / 3.3) * 360; // 0v = 0 degrees, 3.3v = 360 degrees
  }

  /**
   * @brief gets the difference between two angles in degrees
   * @param a the first angle
   * @param b the second angle
   * @return the between the two angles in degrees
   * @note positive means a is larger, negative means b is larger
   */
  private double getDifferenceDegrees(double a, double b) {
    double diff = a - b; // get raw difference between the two angles
    while (diff > 180.0) diff -= 360.0; // adjust to be under 180 degrees
    while (diff < -180.0) diff += 360.0; // adjust to be over -180 degrees
    return diff;
  }
}
