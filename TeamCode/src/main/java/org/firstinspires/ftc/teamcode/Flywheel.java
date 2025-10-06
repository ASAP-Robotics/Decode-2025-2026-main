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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

public class Flywheel {
  private final DcMotorEx flywheel;
  private static final double G = 9.81;
  private final double motorTicksPerRev; // /< ticks per revolution of flywheel motor
  private boolean isEnabled = false; // /< if the flywheel is enabled
  private boolean isActive = true; // /< if the flywheel is active (as opposed to idling)
  private double idleSpeed; // /< the speed (RPM) of the flywheel when idle
  private double targetSpeed = 0; // /< the speed (RPM) the flywheel is targeting
  private double currentSpeed = 0; // /< the latest speed (RPM) of the flywheel
  private double targetDistance = 0; // /< the distance (inches) to the target
  private boolean containsBall = false; // /< if the flywheel has a ball in it that it is shooting
  public org.firstinspires.ftc.teamcode.utils.SimpleTimer
      shotTimer; // /< timer to keep flywheel on while shooting a ball

  // ---- set these to match your robot ----
  private static final double ANGLE_DEG = 45.0; // shooter pitch
  private static final double DELTA_H_IN = 24.0; // goalHeight - releaseHeight (inches)
  private static final double WHEEL_DIAM_IN = 3.78; // flywheel diameter (inches)

  // ---- friction/slip lumped into one factor (0<eff<=1). Start ~0.90 and tune. ----
  private static final double EFFICIENCY = 0.90;

  private final double SPEED_TOLERANCE =
      0.1; // think of as the percentage of the target speed the flywheel needs to reach to be "at

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
    this.flywheel.setZeroPowerBehavior(
        DcMotorEx.ZeroPowerBehavior.FLOAT); // spin freely if zero power (motor stoped)
    this.flywheel.setDirection(DcMotorEx.Direction.FORWARD);
    motorTicksPerRev = this.flywheel.getMotorType().getTicksPerRev(); // get ticks per rev
    this.idleSpeed = idleSpeed; // set the speed of the flywheel at idle
    shotTimer = new SimpleTimer(shotTimeSeconds);
  }

  /**
   * @brief returns if the flywheel is fully up to speed
   * @return true if flywheel is at speed, false if flywheel is below target speed
   * @note workaround TODO: remove workaround and fix
   */
  public boolean isUpToSpeed() {
    update(); // update flywheel
    /*
    return currentSpeed >= (targetSpeed * SPEED_TOLERANCE);
    */
    return true;
  }

  /**
   * @brief returns if a ball is in the flywheel
   * @return true if a ball is in the flywheel, false if the flywheel is empty
   * @note this is basically just a wrapper around a variable that isn't used in any core methods
   */
  public boolean getContainsBall() {
    return containsBall;
  }

  /**
   * @brief sets if a ball is in the flywheel
   * @param containsBall true if a ball is in the flywheel, false if the flywheel is empty
   * @note this is basically just a wrapper around a variable that isn't used in any core methods
   */
  public void setContainsBall(boolean containsBall) {
    this.containsBall = containsBall;
  }

  /**
   * @brief sets if the flywheel is enabled
   * @param isEnabled if the flywheel will be enabled (true = enabled, false = not enabled)
   * @return the previous enabled / disabled state of the flywheel
   */
  public boolean setEnabled(boolean isEnabled) {
    boolean toReturn = this.isEnabled; // get the old enabled state
    this.isEnabled = isEnabled; // set the new enabled state
    this.flywheel.setZeroPowerBehavior(
        isEnabled
            ? DcMotorEx.ZeroPowerBehavior.FLOAT
            : DcMotorEx.ZeroPowerBehavior
                .BRAKE); // set behavior if zero power (motor stoped); if enabled, spin freely, if
    // disabled, brake
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

    if (isEnabled) {
      if (isActive) {
        startMotor(); // set the motor to the correct speed
      } else {
        idleMotor(); // set the motor to idling speeds
      }
    } else {
      stopMotor(); // stop the flywheel
    }
  }

  /**
   * @brief sets the motor to the idle speed
   * @note use setIdleSpeed() to set the idle speed
   */
  private void idleMotor() {
    if (currentSpeed <= idleSpeed) {
      double ticksPerSec = (idleSpeed / 60.0) * motorTicksPerRev;
      flywheel.setVelocity(ticksPerSec); // set the speed using the built-in PID controller
    } else {
      flywheel.setPower(0); // spin freely
    }
  }

  /**
   * @brief sets the motor to the correct speed to hit the target
   * @note use `setTargetDistance()` to set the distance from the target
   */
  private void startMotor() {
    double rpm = rpmForDistance(targetDistance);
    double ticksPerRev = flywheel.getMotorType().getTicksPerRev();
    double ticksPerSec = (rpm / 60.0) * ticksPerRev;
    targetSpeed = rpm; // store target speed
    flywheel.setVelocity(ticksPerSec); // built-in velocity PID
  }

  /**
   * @brief stops the flywheel
   */
  private void stopMotor() {
    flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // brake if zero power
    flywheel.setPower(0);
  }

  /**
   * @brief finds the correct flywheel speed to hit a target a specified distance away
   * @param Rin the target distance in inches
   * @return the correct flywheel RPM to hit the target
   * @note Core math: distance (in) → wheel RPM, including efficiency loss
   */
  private double rpmForDistance(double Rin) {
    // convert to meters
    double Rm = Rin * 0.0254;
    double deltaHm = DELTA_H_IN * 0.0254;
    double wheelDiamM = WHEEL_DIAM_IN * 0.0254;

    double theta = Math.toRadians(ANGLE_DEG);
    double cos = Math.cos(theta), tan = Math.tan(theta);
    double denom = 2.0 * cos * cos * (Rm * tan - deltaHm);
    if (denom <= 0) throw new IllegalArgumentException("Unreachable shot at this distance.");

    double vExit = Math.sqrt(G * Rm * Rm / denom); // ideal exit speed (m/s)

    // ---- Apply efficiency: wheel surface speed must be higher than exit speed. ----
    double wheelSurfaceSpeed = vExit / EFFICIENCY;

    return (60.0 * wheelSurfaceSpeed) / (Math.PI * wheelDiamM); // RPM
  }
}
