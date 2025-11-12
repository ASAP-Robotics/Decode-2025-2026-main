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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utils.MathUtils;

public abstract class Flywheel<T extends Flywheel.LookupTableItem> {
  protected abstract static class LookupTableItem {
    /**
     * @brief gets the distance associated with this lookup table entry
     * @return the distance the values in this lookup table entry were tuned for
     */
    public abstract double getDistance();

    /**
     * @brief gets the flywheel rpm associated with this lookup table entry
     * @return the rpm the flywheel should spin at
     */
    public abstract double getRpm();
  }

  public final DcMotorEx flywheel;
  protected boolean isEnabled = false; // if the flywheel is enabled
  protected boolean isActive = true; // if the flywheel is active (as opposed to idling)
  private double idleSpeed; // the speed (RPM) of the flywheel when idle
  private double targetSpeed = 0; // the speed (RPM) the flywheel is targeting
  private double currentSpeed = 0; // the latest speed (RPM) of the flywheel
  private double targetDistance = 0; // the distance (inches) to the target
  @Deprecated
  private boolean containsBall = false; // if the flywheel has a ball in it that it is shooting
  public double testingSpeed = 2000;
  public final boolean testing;
  public final double MOTOR_TICKS_PER_REV = 28; // ticks per revolution of flywheel motor

  protected T[] LOOKUP_TABLE; // lookup table of distance, rpm, etc.

  private static final double SPEED_TOLERANCE =
      0.95; // think of as the percentage of the target speed the flywheel needs to reach to be "at

  // target speed"

  /**
   * @brief makes an object of the Flywheel class
   * @param motor the motor used for the flywheel
   * @param idleSpeed the speed of the flywheel when idling (RPM)
   */
  public Flywheel(DcMotorEx motor, double idleSpeed) {
    this(motor, idleSpeed, false);
  }

  /**
   * @brief makes an object of the Flywheel class
   * @param motor the motor used for the flywheel
   * @param idleSpeed the speed of the flywheel when idling (RPM)
   * @param testing if the flywheel will be tested (pass false for normal use)
   */
  public Flywheel(DcMotorEx motor, double idleSpeed, boolean testing) {
    this.flywheel = motor;
    this.idleSpeed = idleSpeed; // set the speed of the flywheel at idle
    this.testing = testing;
    this.LOOKUP_TABLE = fillLookupTable();
    // set motor to use speed-based control
    this.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // set motor to spin freely if set to 0% power
    this.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    // set motor to spin the right way
    this.flywheel.setDirection(DcMotor.Direction.REVERSE);
    // quick-and-dirty tuning values, could be updated:
    this.flywheel.setPIDFCoefficients(
        DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 3, 5, 16));
  }

  /**
   * @brief fills the lookup table
   * @return the full lookup table
   */
  protected abstract T[] fillLookupTable();

  /**
   * @brief returns if the flywheel is fully up to speed
   * @return true if flywheel is at speed, false if flywheel is below target speed
   * @note doesn't check the flywheel speed; call update() to update flywheel speed reading
   */
  public boolean isReadyToShoot() {
    return currentSpeed >= (targetSpeed * SPEED_TOLERANCE);
  }

  /**
   * @brief returns if a ball is in the flywheel
   * @return true if a ball is in the flywheel, false if the flywheel is empty
   * @note no actual detection of if the flywheel contains a ball is done
   */
  @Deprecated
  public boolean containsBall() {
    return containsBall;
  }

  /**
   * @brief returns if the ball has left the flywheel since the last check
   * @return true if the turret contained a ball, and the shot timer finished since last call
   * @note no actual detection of if the flywheel contains a ball is done
   */
  @Deprecated
  public boolean ballLeft() {
    if (containsBall) {
      containsBall = false;
      return true;
    }

    return false;
  }

  /**
   * @brief sets that a ball is in the flywheel
   * @note no actual detection of if the flywheel contains a ball is done
   */
  @Deprecated
  public void setContainsBall() {
    containsBall = true;
  }

  /**
   * @brief sets if the flywheel is enabled
   * @param isEnabled if the flywheel will be enabled (true = enabled, false = not enabled)
   * @return the previous enabled / disabled state of the flywheel
   * @note the new value isn't applied until update() is called
   */
  public boolean setEnabled(boolean isEnabled) {
    boolean toReturn = this.isEnabled; // get the old enabled state
    this.isEnabled = isEnabled; // set the new enabled state
    // set motor mode; if enabled, use speed-based control, if disabled, use power-based control
    flywheel.setMode(
        isEnabled ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    return toReturn; // return the old enabled state
  }

  /**
   * @brief returns if the flywheel is enabled
   * @return true if the flywheel is enabled, fals if the flywheel is disabled
   */
  public boolean isEnabled() {
    return isEnabled;
  }

  /**
   * @brief enables the flywheel
   * @return the previous enabled / disabled state of the flywheel
   * @note the new value isn't applied until update() is called
   */
  public boolean enable() {
    return setEnabled(true);
  }

  /**
   * @brief disables the flywheel
   * @return the previous enabled / disabled state of the flywheel
   * @note the new value isn't applied until update() is called
   */
  public boolean disable() {
    return setEnabled(false);
  }

  /**
   * @brief returns if the flywheel is active
   * @return true if the flywheel is active, false if the flywheel isn't active
   */
  public boolean isActive() {
    return isActive;
  }

  /**
   * @brief sets if the flywheel is active (as opposed to idling)
   * @param isActive The new activity state, true if active, false if idling
   * @return the old activation state, true if active, false if idling
   * @note the new value isn't applied until update() is called
   */
  public boolean setActive(boolean isActive) {
    boolean toReturn = this.isActive; // store the old activation state
    this.isActive = isActive; // update if the flywheel is active or idling
    return toReturn; // return the old activation state
  }

  /**
   * @brief activate the flywheel (spin it up to full speed from idle speed)
   * @return the old activation state of the flywheel (true if active, false if idle)
   * @note the new value isn't applied until update() is called
   */
  public boolean activate() {
    return setActive(true);
  }

  /**
   * @brief idles the flywheel (lower it to idle speed from full speed)
   * @return the old activation state of the flywheel (true if active, false if idle)
   * @note the new value isn't applied until update() is called
   */
  public boolean idle() {
    return setActive(false);
  }

  /**
   * @brief sets the idle speed of the flywheel, in RPM
   * @param idleSpeed the speed at which the flywheel should idle
   * @return the old idle speed of the flywheel
   * @note the new value isn't applied until update() is called
   */
  public double setIdleSpeed(double idleSpeed) {
    double toReturn = this.idleSpeed; // store the old idle speed
    this.idleSpeed = idleSpeed; // set the new flywheel idle speed
    return toReturn; // return the old idle speed
  }

  /**
   * @brief returns the speed of the flywheel when idle
   * @return the idle speed of the flywheel, in RPM
   */
  public double getIdleSpeed() {
    return idleSpeed; // return the idle speed
  }

  /**
   * @brief sets the distance to the target
   * @param distance the new distance to the target, in arbitrary units
   * @note we are actually using the percentage of the camera view occupied by the apriltag, instead
   *     of distance
   * @note the new value isn't applied until update() is called
   */
  public void setTargetDistance(double distance) {
    targetDistance = distance; // set the new distance target
  }

  /**
   * @brief returns the distance to the target being used
   * @return the distance to the target, in inches
   */
  public double getTargetDistance() {
    return targetDistance; // return the distance target
  }

  /**
   * @brief updates the flywheel, setting the motor to the correct speed
   * @note call every loop
   */
  public void update() {
    double ticksPerSec =
        this.flywheel.getVelocity(); // get the speed of the motor in ticks per second
    currentSpeed = (ticksPerSec * 60.0) / MOTOR_TICKS_PER_REV; // convert to RPM, store

    if (isEnabled) { // if flywheel is enabled
      if (isActive) { // if flywheel is active
        startMotor(); // set the motor to the correct speed

      } else { // if flywheel is idle
        idleMotor(); // set the motor to idle speed
      }

    } else { // if flywheel is disabled
      stopMotor(); // set the motor to 0% power
    }
  }

  /**
   * @brief sets the motor to the idle speed
   * @note use setIdleSpeed() to set the idle speed
   */
  private void idleMotor() {
    if (currentSpeed - 300 <= idleSpeed) {
      double ticksPerSec = (idleSpeed / 60.0) * MOTOR_TICKS_PER_REV;
      flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // use speed-based control
      flywheel.setVelocity(ticksPerSec); // set the speed using the built-in PID controller

    } else { // if flywheel is going much too fast
      flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // use power-based control
      flywheel.setPower(0); // spin freely
    }
  }

  /**
   * @brief sets the motor to the correct speed to hit the target
   * @note use `setTargetDistance()` to set the distance from the target
   */
  private void startMotor() {
    double rpm = getRPMLookup(targetDistance);
    double ticksPerSec = (rpm / 60.0) * MOTOR_TICKS_PER_REV;
    targetSpeed = rpm; // store target speed

    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // use speed-based control
    flywheel.setVelocity(ticksPerSec); // set the speed using the built-in PIDF controller
  }

  /**
   * @brief stops powered movement of the flywheel
   */
  private void stopMotor() {
    flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // use power-based control
    flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // float if zero power
    flywheel.setPower(0);
  }

  /**
   * @brief gets the RPM for a given distance from the lookup table
   * @param distance the target distance to get flywheel RPM for
   * @return the flywheel RPM for the given distance
   * @note we are actually using the percentage of the camera view occupied by the apriltag, instead
   *     of distance
   */
  protected double getRPMLookup(double distance) {
    if (testing) return testingSpeed; // used for tuning lookup table
    int indexOver = LOOKUP_TABLE.length - 1;
    int indexUnder = 0;
    for (int i = 0; i < LOOKUP_TABLE.length; i++) {
      if (LOOKUP_TABLE[i].getDistance() >= distance) {
        indexOver = i;
        indexUnder = indexOver - 1; // assuming values go from low to high
        break;
      }
    }

    return MathUtils.map(
        distance,
        LOOKUP_TABLE[indexUnder].getDistance(),
        LOOKUP_TABLE[indexOver].getDistance(),
        LOOKUP_TABLE[indexUnder].getRpm(),
        LOOKUP_TABLE[indexOver].getRpm());
  }
}
