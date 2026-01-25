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

package org.firstinspires.ftc.teamcode.hardware.motors;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.Follower;

/**
 * Class to contain a motor that is set to a given angle setpoint, which can be homed via a button
 * pressed by a cam
 *
 * @note assumes that the break beam is placed at 0 degrees
 */
public class HomableRotator implements System {
  public enum State {
    NORMAL,
    HOMING,
    UNINITIALIZED
  }

  protected static final double HOMING_INCREMENT_SIZE = 1; // degrees

  protected final Motor motor;
  protected final TouchSensor sensor;
  protected final PIDController motorController;
  protected final Follower motorSimulation;
  protected final boolean inverted;
  protected State state = State.UNINITIALIZED;
  protected boolean homed = false;
  protected double currentAngle = 0;
  protected double targetAngle = 0;

  public HomableRotator(
      Motor motor,
      TouchSensor sensor,
      double kp,
      double ki,
      double kd,
      double tolerance,
      boolean inverted) {
    this.motor = motor;
    this.sensor = sensor;
    this.motorController = new PIDController(kp, ki, kd);
    this.motorController.setTolerance(tolerance);
    this.motor.setInverted(inverted);
    this.motor.setRunMode(Motor.RunMode.RawPower);
    this.motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    this.motor.set(0);
    this.motorSimulation =
        new Follower(0, 0, tolerance, this.motor.getMaxRPM() / 12); // tune 12 (1/2 of max speed)
    this.inverted = inverted;
  }

  public void start() {
    state = State.NORMAL;
    motor.set(0);
    motor.stopAndResetEncoder();
    motorController.reset();
    setTargetAngle(0);
    motorController.setSetPoint(targetAngle);
  }

  public void update() {
    if (state == State.UNINITIALIZED) return;
    measureCurrentAngle();
    motor.set(
        Range.clip(motorController.calculate(getCurrentAngle() * (inverted ? -1 : 1)), -1, 1));
    if (state == State.HOMING && motorController.atSetPoint()) {
      if (sensor.isPressed()) {
        homed = true;
        state = State.NORMAL;
        motor.set(0);
        motor.stopAndResetEncoder();
        motorController.reset();
        setTargetAngle(0);

      } else {
        changeTargetAngle(HOMING_INCREMENT_SIZE);
      }
    }
  }

  /**
   * Sets the target angle of the motor (can be greater than 360)
   *
   * @param degrees the new target angle
   */
  public void setAngle(double degrees) {
    if (state != State.NORMAL) return;
    setTargetAngle(degrees);
  }

  /**
   * Sets the target angle of the motor (can be greater than 360)
   *
   * @note Not safe to use while homing, prefer setAngle() instead
   * @param degrees the new target angle
   */
  protected void setTargetAngle(double degrees) {
    if (Double.isNaN(degrees) || Double.isInfinite(degrees)) return;
    targetAngle = degrees;
    motorController.setSetPoint(degrees);
    motorSimulation.setTarget(degrees);
  }

  /**
   * Changes the target angle of the motor b a given amount
   *
   * @param change the amount to change the target angle by
   */
  public void changeTargetAngle(double change) {
    setTargetAngle(getTargetAngle() + change);
  }

  /**
   * Gets the target angle of the motor
   *
   * @return the target angle
   */
  public double getTargetAngle() {
    return targetAngle;
  }

  /**
   * Gets the target angle, normalized to 0-360 degrees
   *
   * @return the normalized target angle
   */
  public double getNormalizedTargetAngle() {
    return AngleUnit.normalizeDegrees(getTargetAngle()) + 180;
  }

  /** Measures the current angle of the motor */
  private void measureCurrentAngle() {
    double angle = motor.getCurrentPosition() / motor.getCPR() * (inverted ? -360 : 360);
    currentAngle = Double.isNaN(angle) || Double.isInfinite(angle) ? 0 : angle;
  }

  /**
   * Gets the last read angle of the motor
   *
   * @return the motor's angle (can be over 360 or under 0 degrees), or 0 if angle is invalid
   */
  public double getCurrentAngle() {
    return currentAngle;
  }

  /**
   * Gets the current angle of the motor, normalized to 0-360 degrees
   *
   * @return the normalized value of the motor's angle
   */
  public double getNormalizedCurrentAngle() {
    return AngleUnit.normalizeDegrees(getCurrentAngle()) + 180;
  }

  /**
   * Gets the distance (in degrees) between the given angle and the current angle. Basically, if the
   * given angle was set as the target, how far would the motor move?
   *
   * @param angle the "target" to measure the distance from
   * @return the distance from the given target, and the current angle
   * @note returns positive infinity on invalid parameters
   */
  public double getAngleTravel(double angle) {
    if (Double.isNaN(angle) || Double.isInfinite(angle)) return Double.POSITIVE_INFINITY;
    return Math.abs(getCurrentAngle() - angle);
  }

  /** Starts homing the motor */
  public void home() {
    homed = false;
    targetAngle = 0;
    state = State.HOMING;
    motor.set(0);
    motor.stopAndResetEncoder();
    motorController.reset();
    motorController.setSetPoint(targetAngle);
  }

  /**
   * Gets if the motor is at it's target angle
   *
   * @return true if at target, false otherwise
   */
  public boolean atTarget() {
    return state == State.NORMAL && motorController.atSetPoint();
  }

  /**
   * Gets if the motor is homed
   *
   * @return true if homed, false otherwise
   */
  public boolean isHomed() {
    return homed;
  }

  /**
   * Sets the tolerance of the motor (how close to the target is "at target")
   *
   * @param degrees tolerance
   */
  public void setTolerance(double degrees) {
    motorController.setTolerance(degrees);
  }

  public SystemReport getStatus() {
    org.firstinspires.ftc.teamcode.types.SystemStatus status =
        state == State.NORMAL && motorSimulation.isAtTarget() && !motorController.atSetPoint()
            ? SystemStatus.INOPERABLE
            : SystemStatus.NOMINAL;

    return new SystemReport(status);
  }

  /**
   * Sets the PID tuning values for the PID controller
   *
   * @param kp proportional
   * @param ki integral
   * @param kd derivative
   */
  public void setPID(double kp, double ki, double kd) {
    motorController.setPID(kp, ki, kd);
  }

  /**
   * Sets the P tuning value for the PID controller
   *
   * @param kp proportional
   */
  public void setP(double kp) {
    motorController.setP(kp);
  }

  /**
   * Sets the I tuning value for the PID controller
   *
   * @param ki integral
   */
  public void setI(double ki) {
    motorController.setI(ki);
  }

  /**
   * Sets the D tuning value for the PID controller
   *
   * @param kd derivative
   */
  public void setD(double kd) {
    motorController.setD(kd);
  }

  /**
   * Gets the PID tuning values from the PID controller
   *
   * @return PID values
   */
  public double[] getPID() {
    return motorController.getCoefficients();
  }

  /**
   * Gets the P tuning value from the PID controller
   *
   * @return proportional
   */
  public double getP() {
    return motorController.getP();
  }

  /**
   * Gets the I tuning value from the PID controller
   *
   * @return integral
   */
  public double getI() {
    return motorController.getI();
  }

  /**
   * Gets the D tuning value from the PID controller
   *
   * @return derivative
   */
  public double getD() {
    return motorController.getD();
  }
}
