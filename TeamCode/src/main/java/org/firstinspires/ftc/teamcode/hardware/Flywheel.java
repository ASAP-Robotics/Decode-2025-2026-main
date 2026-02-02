/*
 * Copyright 2025-2026 ASAP Robotics (FTC Team 22029)
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
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.Follower;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.jetbrains.annotations.TestOnly;

public abstract class Flywheel<T extends Flywheel.LookupTableItem> implements System {
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

  public enum ControlMode {
    PIDF,
    BANG_BANG
  }

  private static final double RPM_TARGET_TOLERANCE = 10;
  private final double MOTOR_TICKS_PER_REV = 28; // ticks per revolution of flywheel motor
  protected final DcMotorEx flywheel;
  protected Follower speedSimulation; // simulation of flywheel speed
  protected SystemStatus flywheelStatus; // status of the flywheel
  protected ControlMode controlMode = ControlMode.PIDF;
  protected boolean isEnabled = false; // if the flywheel is enabled
  protected boolean isActive = true; // if the flywheel is active (as opposed to idling)
  private double idleSpeed; // the speed (RPM) of the flywheel when idle
  private double targetSpeed = 0; // the speed (RPM) the flywheel is targeting
  private double currentSpeed = 0; // the latest speed (RPM) of the flywheel
  private double targetDistance = 0; // the distance (inches) to the target
  private double lastSetSpeed = 0; // the last RPM the flywheel was set to spin at
  private DcMotor.RunMode flywheelRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
  protected double testingSpeed = 2000;
  protected boolean testing = false;

  protected T[] LOOKUP_TABLE; // lookup table of distance, rpm, etc.

  /**
   * @brief makes an object of the Flywheel class
   * @param motor the motor used for the flywheel
   * @param idleSpeed the speed of the flywheel when idling (RPM)
   */
  public Flywheel(DcMotorEx motor, double idleSpeed) {
    this.speedSimulation = new Follower(0, 0, 5, 100); // tune 100
    this.flywheel = motor;
    this.idleSpeed = idleSpeed; // set the speed of the flywheel at idle
    this.LOOKUP_TABLE = fillLookupTable();
    this.flywheel.setVelocityPIDFCoefficients(300, 1, 0, 16);
    // set motor to use speed-based control
    this.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // set motor to spin freely if set to 0% power
    this.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    // set motor to spin the right way
    this.flywheel.setDirection(DcMotor.Direction.REVERSE);
  }

  /**
   * @brief fills the lookup table
   * @return the full lookup table
   */
  protected abstract T[] fillLookupTable();

  public SystemReport getStatus() {
    return new SystemReport(flywheelStatus);
  }

  /**
   * @brief returns if the flywheel is fully up to speed
   * @return true if flywheel is at speed, false if flywheel is below target speed
   * @note doesn't check the flywheel speed; call update() to update flywheel speed reading
   */
  public boolean isReadyToShoot() {
    return isEnabled && isActive && (isAtSpeed() || speedSimulation.isAtTarget());
  }

  /**
   * @brief checks if the measured flywheel speed is at target
   * @return true if flywheel is actually at speed, false otherwise
   */
  protected boolean isAtSpeed() {
    return currentSpeed >= (targetSpeed - 140) && currentSpeed <= (targetSpeed + 140);
  }

  /**
   * @brief manually sets RPM, use to tune lookup table
   * @param rpm the speed to spin the flywheel at
   */
  @TestOnly
  protected void overrideRpm(double rpm) {
    testing = true;
    testingSpeed = rpm;
  }

  /**
   * Sets the control mode of the flywheel when active
   *
   * @param mode the control mode to use when active
   * @note the idea is thet flywheel switches to bang-bang when shooting, and PIDF the rest of the
   *     time
   */
  public void setControlMode(ControlMode mode) {
    controlMode = mode;
  }

  /**
   * @param isEnabled if the flywheel will be enabled (true = enabled, false = not enabled)
   * @brief sets if the flywheel is enabled
   * @note the new value isn't applied until update() is called
   */
  public void setEnabled(boolean isEnabled) {
    this.isEnabled = isEnabled; // set the new enabled state
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
   * @note the new value isn't applied until update() is called
   */
  public void enable() {
    setEnabled(true);
  }

  /**
   * @brief disables the flywheel
   * @note the new value isn't applied until update() is called
   */
  public void disable() {
    setEnabled(false);
  }

  /**
   * @brief returns if the flywheel is active
   * @return true if the flywheel is active, false if the flywheel isn't active
   */
  public boolean isActive() {
    return isActive;
  }

  /**
   * @param isActive The new activity state, true if active, false if idling
   * @brief sets if the flywheel is active (as opposed to idling)
   * @note the new value isn't applied until update() is called
   */
  public void setActive(boolean isActive) {
    this.isActive = isActive; // update if the flywheel is active or idling
  }

  /**
   * @brief activate the flywheel (spin it up to full speed from idle speed)
   * @note the new value isn't applied until update() is called
   */
  public void activate() {
    setActive(true);
  }

  /**
   * @brief idles the flywheel (lower it to idle speed from full speed)
   * @note the new value isn't applied until update() is called
   */
  public void idle() {
    setActive(false);
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

    speedSimulation.setTarget(targetSpeed);

    flywheelStatus =
        isEnabled && isActive && !isAtSpeed() && speedSimulation.isAtTarget()
            ? SystemStatus.FALLBACK
            : SystemStatus.NOMINAL;
  }

  /**
   * @brief sets the motor to the idle speed
   * @note use setIdleSpeed() to set the idle speed
   */
  private void idleMotor() {
    targetSpeed = idleSpeed;
    if (currentSpeed - 200 <= idleSpeed) {
      if (flywheelRunMode != DcMotor.RunMode.RUN_USING_ENCODER) {
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // use speed-based control
        flywheelRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
      }

      if (Math.abs(idleSpeed - lastSetSpeed) > RPM_TARGET_TOLERANCE) {
        double ticksPerSec = (idleSpeed / 60.0) * MOTOR_TICKS_PER_REV;
        flywheel.setVelocity(ticksPerSec); // set the speed using the built-in PID controller
        lastSetSpeed = idleSpeed;
      }

    } else { // if flywheel is going much too fast
      if (flywheelRunMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // use power-based control
        flywheelRunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
      }

      if (lastSetSpeed != 0) {
        flywheel.setPower(0); // spin freely
        lastSetSpeed = 0;
      }
    }
  }

  /**
   * @brief sets the motor to the correct speed to hit the target
   * @note use `setTargetDistance()` to set the distance from the target
   */
  private void startMotor() {
    double rpm = getRPMLookup(targetDistance);
    targetSpeed = rpm; // store target speed
    DcMotor.RunMode targetRunMode =
        controlMode == ControlMode.PIDF
            ? DcMotor.RunMode.RUN_USING_ENCODER
            : DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    if (flywheelRunMode != targetRunMode) {
      flywheel.setMode(targetRunMode);
      flywheelRunMode = targetRunMode;
    }

    switch (controlMode) {
      case BANG_BANG:
        if (currentSpeed > rpm) {
          flywheel.setPower(0);

        } else {
          flywheel.setPower(1);
        }
        lastSetSpeed = Double.NEGATIVE_INFINITY; // so speed will be set when switching back to PIDF
        break;

      case PIDF:
      default:
        if (Math.abs(rpm - lastSetSpeed) > RPM_TARGET_TOLERANCE) {
          double ticksPerSec = (rpm / 60.0) * MOTOR_TICKS_PER_REV;
          flywheel.setVelocity(ticksPerSec); // set the speed using the built-in PIDF controller
          lastSetSpeed = rpm;
        }
        break;
    }
  }

  /**
   * @brief stops powered movement of the flywheel
   */
  private void stopMotor() {
    targetSpeed = 0;

    if (flywheelRunMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
      flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // use power-based control
      flywheelRunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    }

    if (lastSetSpeed != 0) {
      flywheel.setPower(0);
      lastSetSpeed = 0;
    }
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
    try {
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
    } catch (Exception e) { // most probably if distance is outside of lookup table
      return 0;
    }
  }
}
