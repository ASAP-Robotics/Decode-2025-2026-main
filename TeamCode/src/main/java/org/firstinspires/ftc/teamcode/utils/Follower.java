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

package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @brief simple class to model a system where a value moves to a target linearly over time
 */
public class Follower {
  protected double target; // the target position of the modeled system
  protected double value; // the current position of the modeled system
  protected final double tolerance; // the tolerance of the modeled system
  protected final double unitsPerSecond; // the speed of the modeled system, in units per second
  protected ElapsedTime updateTime; // the time sine the last update

  /**
   * @brief makes a new Follower
   * @param value the initial value of the modeled system
   * @param target the initial target of the modeled system
   * @param tolerance the amount the value can differ from the target and be "at target"
   * @param unitsPerSecond the speed at which the modeled system moves, in units per second
   */
  public Follower(double value, double target, double tolerance, double unitsPerSecond) {
    this.value = value;
    this.target = target;
    this.tolerance = tolerance;
    this.unitsPerSecond = unitsPerSecond;
    this.updateTime = new ElapsedTime();
  }

  /**
   * @brief gets the tolerance of the modeled system
   * @return the tolerance of the system
   */
  public double getTolerance() {
    return tolerance;
  }

  /**
   * @brief sets the target value of the modeled system
   * @param target the target value of the modeled system
   */
  public void setTarget(double target) {
    this.target = target;
  }

  /**
   * @brief gets the target value of the modeled system
   * @return the target value of the system
   */
  public double getTarget() {
    return target;
  }

  /**
   * @brief gets if the modeled system is at it's target value
   * @return true if at target, false otherwise
   */
  public boolean isAtTarget() {
    return Math.abs(getTarget() - getValue()) <= getTolerance();
  }

  /**
   * Overrides the value of the modeled system
   *
   * @note this is an advanced feature, and usually isn't needed. use with discretion
   * @param value the new value of the modeled system
   */
  public void setValue(double value) {
    this.value = value;
  }

  /**
   * @brief gets the current value of the system
   * @return the current value of the system
   */
  public double getValue() {
    update();
    return value;
  }

  /**
   * @brief updates the current value of the modeled system
   */
  protected void update() {
    double deltaTime = updateTime.seconds();
    updateTime.reset();
    double error = target - value;
    double maxChange = unitsPerSecond * deltaTime;
    value += Math.copySign(Math.min(Math.abs(error), maxChange), error);
  }
}
