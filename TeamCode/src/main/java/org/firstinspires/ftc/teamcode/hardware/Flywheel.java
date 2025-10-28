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
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

public class Flywheel {
  private final DcMotorEx flywheel;
  private final double motorTicksPerRev; // ticks per revolution of flywheel motor
  protected boolean isEnabled = false; // if the flywheel is enabled
  protected boolean isActive = true; // if the flywheel is active (as opposed to idling)
  private double idleSpeed; // the speed (RPM) of the flywheel when idle
  private double targetSpeed = 0; // the speed (RPM) the flywheel is targeting
  private double currentSpeed = 0; // the latest speed (RPM) of the flywheel
  private double targetDistance = 0; // the distance (inches) to the target
  private boolean containsBall = false; // if the flywheel has a ball in it that it is shooting
  // timer to keep flywheel on while shooting a ball
  private final org.firstinspires.ftc.teamcode.utils.SimpleTimer shotTimer;

  // these arrays constitute a lookup table for finding the correct flywheel RPM for a distance
  // the distance values need to be in ascending order, or things will break
  // the two array MUST be the same length, or things will break
  private static final double[] DISTANCES = {
    5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
  }; // placeholder values TODO: tune
  private static final double[] RPMs = {
    1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250, 4500, 4750, 5000,
    5250, 5500, 5750, 6000
  }; // placeholder values TODO: tune

  private final double SPEED_TOLERANCE =
      0.95; // think of as the percentage of the target speed the flywheel needs to reach to be "at

  // target speed"

  /**
   * @brief makes an object of the Flywheel class with and idle speed of 500 RPM
   * @param motor the motor used for the flywheel
   */
  public Flywheel(DcMotorEx motor) {
    this(
        motor, 500,
        1); // make a Flywheel with an idle speed of 500 RPM and a shot time of one second
  }

  /**
   * @brief makes an object of the Flywheel class
   * @param motor the motor used for the flywheel
   * @param idleSpeed the speed of the flywheel when idling (RPM)
   */
  public Flywheel(DcMotorEx motor, double idleSpeed, double shotTimeSeconds) {
    this.flywheel = motor;
    // set motor to use speed-based control
    this.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // set motor to spin freely if set to 0% power
    this.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    // set motor to spin forwards
    this.flywheel.setDirection(DcMotor.Direction.FORWARD);
    motorTicksPerRev = this.flywheel.getMotorType().getTicksPerRev(); // get ticks per rev
    this.idleSpeed = idleSpeed; // set the speed of the flywheel at idle
    shotTimer = new SimpleTimer(shotTimeSeconds);
  }

  /**
   * @brief returns if the flywheel is fully up to speed
   * @return true if flywheel is at speed, false if flywheel is below target speed
   */
  public boolean isUpToSpeed() {
    update(); // update flywheel
    return currentSpeed >= (targetSpeed * SPEED_TOLERANCE);
  }

  /**
   * @brief returns if a ball is in the flywheel
   * @return true if a ball is in the flywheel, false if the flywheel is empty
   * @note no actual detection of if the flywheel contains a ball is done
   */
  public boolean containsBall() {
    return containsBall;
  }

  /**
   * @brief returns if the ball has left the flywheel since the last check
   * @return true if the turret contained a ball, and the shot timer finished since last call
   * @note no actual detection of if the flywheel contains a ball is done
   */
  public boolean ballLeft() {
    if (containsBall && shotTimer.isFinished()) {
      containsBall = false;
      return true;
    }

    return false;
  }

  /**
   * @brief sets that a ball is in the flywheel
   * @note no actual detection of if the flywheel contains a ball is done
   */
  public void setContainsBall() {
    // start shot timer if the turret now contains a ball
    if (!containsBall) shotTimer.start();
    containsBall = true;
  }

  /**
   * @brief sets if the flywheel is enabled
   * @param isEnabled if the flywheel will be enabled (true = enabled, false = not enabled)
   * @return the previous enabled / disabled state of the flywheel
   */
  public boolean setEnabled(boolean isEnabled) {
    boolean toReturn = this.isEnabled; // get the old enabled state
    this.isEnabled = isEnabled; // set the new enabled state
    // set motor mode; if enabled, use speed-based control, if disabled, use power-based control
    flywheel.setMode(
        isEnabled ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    update(); // apply any changes
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
   */
  public boolean enable() {
    return setEnabled(true);
  }

  /**
   * @brief disables the flywheel
   * @return the previous enabled / disabled state of the flywheel
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
   */
  public boolean setActive(boolean isActive) {
    boolean toReturn = this.isActive; // store the old activation state
    this.isActive = isActive; // update if the flywheel is active or idling
    update(); // apply any changes
    return toReturn; // return the old activation state
  }

  /**
   * @brief activate the flywheel (spin it up to full speed from idle speed)
   * @return the old activation state of the flywheel (true if active, false if idle)
   */
  public boolean activate() {
    return setActive(true);
  }

  /**
   * @brief idles the flywheel (lower it to idle speed from full speed)
   * @return the old activation state of the flywheel (true if active, false if idle)
   */
  public boolean idle() {
    return setActive(false);
  }

  public double setIdleSpeed(double idleSpeed) {
    double toReturn = this.idleSpeed; // store the old idle speed
    this.idleSpeed = idleSpeed; // set the new flywheel idle speed
    update(); // apply any changes
    return toReturn; // return the old idle speed
  }

  /**
   * returns the speed of the flywheel when idle
   *
   * @return the idle speed of the flywheel, in RPM
   */
  public double getIdleSpeed() {
    return idleSpeed; // return the idle speed
  }

  /**
   * @brief sets the distance to the target
   * @param distInches the new distance to the target, in inches
   * @return the old distance to the target, in inches
   */
  public double setTargetDistance(double distInches) {
    double toReturn = targetDistance; // store the old distance target
    targetDistance = distInches; // set the new distance target
    update(); // apply any changes
    return toReturn; // return the old distance target
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
    currentSpeed = (ticksPerSec * 60.0) / motorTicksPerRev; // convert to RPM, store

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
    if (currentSpeed <= idleSpeed) { // if flywheel is going too slow
      double ticksPerSec = (idleSpeed / 60.0) * motorTicksPerRev;
      flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // use speed-based control
      flywheel.setVelocity(ticksPerSec); // set the speed using the built-in PID controller

    } else { // if flywheel is going too fast
      flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // use power-based control
      flywheel.setPower(0); // spin freely
    }
  }

  /**
   * @brief sets the motor to the correct speed to hit the target
   * @note use `setTargetDistance()` to set the distance from the target
   */
  private void startMotor() {
    double rpm = rpmForDistance(targetDistance);
    double ticksPerSec = (rpm / 60.0) * motorTicksPerRev;
    targetSpeed = rpm; // store target speed

    if (currentSpeed <= rpm) { // if flywheel is going too slow
      flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // use speed-based control
      flywheel.setVelocity(ticksPerSec); // set the speed using the built-in PID controller

    } else { // if flywheel is going too fast
      flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // use power-based control
      flywheel.setPower(0); // spin freely
    }
  }

  /**
   * @brief stops powered movement of the flywheel
   */
  private void stopMotor() {
    flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // use power-based control
    flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // brake if zero power
    flywheel.setPower(0);
  }

  /**
   * @brief finds the correct flywheel speed to hit a target a specified distance away
   * @param Rin the target distance in inches
   * @return the correct flywheel RPM to hit the target
   * @note Core math: distance (in) → wheel RPM, including efficiency loss
   */
  private double rpmForDistance(double Rin) {
    return getRPMLookup(Rin);
  }

  /**
   * @brief gets the RPM for a given distance from the lookup table
   * @param distanceInches the target distance to get flywheel RPM for
   * @return the flywheel RPM for the given distance
   */
  private double getRPMLookup(double distanceInches) {
    int indexOver = DISTANCES.length - 1;
    int indexUnder = 0;
    for (int i = 0; i < DISTANCES.length; i++) {
      if (DISTANCES[i] >= distanceInches) {
        indexOver = i;
        indexUnder = indexOver - 1; // assuming values go from low to high
        break;
      }
    }

    double toReturn =
        MathUtils.map(
            distanceInches,
            DISTANCES[indexUnder],
            DISTANCES[indexOver],
            RPMs[indexUnder],
            RPMs[indexOver]);

    return toReturn;
  }
}
