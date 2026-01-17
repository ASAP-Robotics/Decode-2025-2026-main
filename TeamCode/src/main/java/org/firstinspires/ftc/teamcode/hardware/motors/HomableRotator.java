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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.sensors.BreakBeam;
import org.firstinspires.ftc.teamcode.utils.Follower;

/**
 * Class to contain a motor that is set to a given angle setpoint,
 * which can be homed via a break beam sensor
 */
public class HomableRotator {
  public enum State {
    NORMAL,
    HOMING,
    UNINITIALIZED
  }

  protected static final double HOMING_INCREMENT_SIZE = 5;

  protected final Motor motor;
  protected final BreakBeam sensor;
  protected final PIDController motorController;
  protected final Follower angleSimulation;
  protected State state = State.UNINITIALIZED;
  protected double targetAngle = 0;

  public HomableRotator(
      Motor motor,
      BreakBeam sensor,
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
    this.angleSimulation =
        new Follower(0, 0, tolerance, this.motor.getMaxRPM() / 24); // tune 24 (1/4 of max speed)
  }

  public void start() {
    state = State.NORMAL;
    motor.stopAndResetEncoder();
    motorController.reset();
    setTargetAngle(0);
    motorController.setSetPoint(targetAngle);
    motor.set(0);
  }

  public void update() {
    if (state == State.UNINITIALIZED) return;
    motor.set(motorController.calculate(motor.getCurrentPosition() / motor.getCPR() * 360));
    if (state == State.HOMING && atTarget()) {
      if (sensor.isBroken()) {
        state = State.NORMAL;
        motor.set(0);
        motor.stopAndResetEncoder();
        motorController.reset();
        setTargetAngle(0);

      } else {
        targetAngle += HOMING_INCREMENT_SIZE;
      }
    }
  }

  /**
   * Sets the target angle of the motor (can be greater than 360)
   * @param degrees the new target angle
   */
  public void setTargetAngle(double degrees) {
    if (Double.isNaN(degrees) || state != State.NORMAL) return;
    targetAngle = degrees;
    motorController.setSetPoint(degrees);
    angleSimulation.setTarget(degrees);
  }

  /**
   * Gets the target angle of the motor
   * @return the target angle
   */
  public double getTargetAngle() {
    return targetAngle;
  }

  /**
   * Gets the target angle, normalized to 0-360 degrees
   * @return the normalized target angle
   */
  public double getNormalizedTargetAngle() {
    return AngleUnit.normalizeDegrees(getTargetAngle());
  }

  /**
   * Starts homing the motor
   */
  public void home() {
    setTargetAngle(0);
    state = State.HOMING;
    motor.set(0);
    motor.stopAndResetEncoder();
    motorController.reset();
    motorController.setSetPoint(targetAngle);
  }

  /**
   * Gets if the motor is at it's target angle
   * @return true if at target, false otherwise
   */
  public boolean atTarget() {
    return motorController.atSetPoint();
  }

  /**
   * Sets the tolerance of the motor (how close to the target is "at target")
   * @param degrees tolerance
   */
  public void setTolerance(double degrees) {
    motorController.setTolerance(degrees);
  }

  /**
   * Sets the PID tuning values for the PID controller
   * @param kp proportional
   * @param ki integral
   * @param kd derivative
   */
  public void setPID(double kp, double ki, double kd) {
    motorController.setPID(kp, ki, kd);
  }

  /**
   * Sets the P tuning value for the PID controller
   * @param kp proportional
   */
  public void setP(double kp) {
    motorController.setP(kp);
  }

  /**
   * Sets the I tuning value for the PID controller
   * @param ki integral
   */
  public void setI(double ki) {
    motorController.setI(ki);
  }

  /**
   * Sets the D tuning value for the PID controller
   * @param kd derivative
   */
  public void setD(double kd) {
    motorController.setD(kd);
  }

  /**
   * Gets the PID tuning values from the PID controller
   * @return PID values
   */
  public double[] getPID() {
    return motorController.getCoefficients();
  }

  /**
   * Gets the P tuning value from the PID controller
   * @return proportional
   */
  public double getP() {
    return motorController.getP();
  }

  /**
   * Gets the I tuning value from the PID controller
   * @return integral
   */
  public double getI() {
    return motorController.getI();
  }

  /**
   * Gets the D tuning value from the PID controller
   * @return derivative
   */
  public double getD() {
    return motorController.getD();
  }
}
