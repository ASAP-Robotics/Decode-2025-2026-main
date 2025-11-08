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

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class Turret extends Flywheel<Turret.LookupTableItem> {
  protected static class LookupTableItem extends Flywheel.LookupTableItem {
    protected final double distance;
    protected final double rpm;
    protected final double angle;

    public LookupTableItem(double distance, double rpm, double angle) {
      this.distance = distance;
      this.rpm = rpm;
      this.angle = angle;
    }

    public double getDistance() {
      return distance;
    }

    public double getRpm() {
      return rpm;
    }

    public double getAngle() {
      return angle;
    }
  }
  // number of teeth on the gear attached to the turret
  public static final double TURRET_GEAR_TEETH = 121;
  // number of teeth on the gear attached to the motor
  private static final double MOTOR_GEAR_TEETH = 24;

  private final DcMotorEx rotator;
  private final Servo hoodServo;
  private final double ticksPerDegree;
  private double targetHorizontalAngleDegrees; // target angle for side-to-side turret movement
  private double targetVerticalAngleDegrees; // target angle for up-and-down turret movement

  public Turret(DcMotorEx flywheelMotor, DcMotorEx rotator, Servo hoodServo, double idleSpeed) {
    super(flywheelMotor, idleSpeed);
    this.rotator = rotator;
    this.hoodServo = hoodServo;
    this.rotator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    this.rotator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    this.rotator.setTargetPosition(0); // placeholder
    this.rotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    this.ticksPerDegree = this.rotator.getMotorType().getTicksPerRev() / 360;
  }

  public Turret(
      DcMotorEx flywheelMotor,
      DcMotorEx rotator,
      Servo hoodServo) {
    this(flywheelMotor, rotator, hoodServo, 500);
  }

  /**
   * @brief fills the lookup table
   * @return the full lookup table
   */
  protected LookupTableItem[] fillLookupTable() {
    return new LookupTableItem[] {
        new LookupTableItem(0.5, 6000, 0),
        new LookupTableItem(0.45, 5500, 1),
        new LookupTableItem(0.4, 5000, 2),
        new LookupTableItem(0.35, 4500, 3),
        new LookupTableItem(0.3, 4000, 4),
        new LookupTableItem(0.25, 3500, 5),
        new LookupTableItem(0.2, 3000, 6),
        new LookupTableItem(0.15, 2500, 7),
        new LookupTableItem(0.1, 2000, 8),
        new LookupTableItem(0.05, 1500, 9),
    }; // placeholder values; TODO: tune
  }

  /**
   * @brief updates the turret
   * @note call every loop
   */
  @Override
  public void update() {
    super.update();
    hoodServo.setPosition(targetVerticalAngleDegrees); // this might need updating
    double motorDegrees = turretDegreesToMotorDegrees(targetHorizontalAngleDegrees);
    rotator.setTargetPosition((int) (motorDegrees * ticksPerDegree));
  }

  /**
   * @brief sets the distance to the target
   * @param distance the new distance to the target, in arbitrary units
   * @note we are actually using the percentage of the camera view occupied by the apriltag, instead of distance
   */
  @Override
  public void setTargetDistance(double distance) {
    setVerticalAngle(getAngleLookup(distance));
    super.setTargetDistance(distance); // this updates
  }

  /**
   * @brief sets the side-to-side angle of the turret in degrees
   * @param degrees the number of degrees from straight to move the turret
   * @return the old horizontal angle of the turret
   */
  public double setHorizontalAngle(double degrees) {
    double toReturn = targetHorizontalAngleDegrees;
    targetHorizontalAngleDegrees = degrees;
    update();
    return toReturn;
  }

  /**
   * @brief returns the target side-to-side angle of the turret
   * @return the target horizontal angle of the turret
   */
  public double getTargetHorizontalAngleDegrees() {
    return targetHorizontalAngleDegrees;
  }

  /**
   * @brief returns the current side-to-side angle of the turret
   * @return the current horizontal angle of the turret
   */
  public double getHorizontalAngleDegrees() {
    double ticks = rotator.getCurrentPosition(); // get motor position in ticks
    double motorDegrees = ticks / ticksPerDegree; // convert motor position to degrees
    return motorDegreesToTurretDegrees(motorDegrees); // convert motor position to turret position
  }

  /**
   * @brief sets the angle of the servo in degrees
   * @param degrees the number of degrees to move the servo to
   * @note doesn't update the turret
   */
  protected void setVerticalAngle(double degrees) {
    targetVerticalAngleDegrees = degrees;
  }

  /**
   * @brief returns the target up-and-down angle of the turret
   * @return the target vertical angle of the turret
   */
  public double getTargetVerticalAngleDegrees() {
    return targetVerticalAngleDegrees;
  }

  /**
   * @brief finds the number of degrees the motor needs to turn for the turret to turn some amount
   * @param turretDegrees the number of degrees the turret should move by
   * @return the number of degrees the motor should turn to get the turret to move correctly
   */
  private double turretDegreesToMotorDegrees(double turretDegrees) {
    return turretDegrees * (TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH);
  }

  /**
   * @brief finds the number of degrees the turret needs to turn for the motor to turn some amount
   * @param motorDegrees the number of degrees the motor should move by
   * @return the number of degrees the turret should turn for the motor to move by the supplied
   *     amount
   */
  private double motorDegreesToTurretDegrees(double motorDegrees) {
    return motorDegrees / (TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH);
  }

  /**
   * @brief gets the angle for a given distance from the lookup table
   * @param distance the distance to get the angle for
   * @return servo angle for the given distance
   * @note we are actually using the percentage of the camera view occupied by the apriltag, instead of distance
   */
  protected double getAngleLookup(double distance) {
    int indexOver = LOOKUP_TABLE.length - 1;
    int indexUnder = 0;
    for (int i = 0; i < LOOKUP_TABLE.length; i++) {
      if (LOOKUP_TABLE[i].getDistance() >= distance) {
        indexOver = i;
        indexUnder = indexOver - 1; // assuming values go from low to high
        break;
      }
    }

    return MathUtils.map(distance,
        LOOKUP_TABLE[indexUnder].getDistance(),
        LOOKUP_TABLE[indexOver].getDistance(),
        LOOKUP_TABLE[indexUnder].getAngle(),
        LOOKUP_TABLE[indexOver].getAngle());
  }
}
