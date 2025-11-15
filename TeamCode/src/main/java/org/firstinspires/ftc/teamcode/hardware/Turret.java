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

import static org.firstinspires.ftc.teamcode.utils.MathUtils.map;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;

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
  public static final double TURRET_GEAR_TEETH = 120;
  // number of teeth on the gear attached to the motor
  private static final double MOTOR_GEAR_TEETH = 24;
  // amount horizontal angle can go over 180 or under -180 degrees before wrapping
  private static final double HORIZONTAL_HYSTERESIS = 10;

  public final Motor rotator;
  public final Axon hoodServo;
  private final double ticksPerDegree;
  private double targetHorizontalAngleDegrees = 0; // target angle for side-to-side turret movement
  private double targetVerticalAngleDegrees = 90; // target angle for up-and-down turret movement

  public Turret(
      DcMotorEx flywheelMotor, Motor rotator, Axon hoodServo, double idleSpeed, boolean testing) {
    super(flywheelMotor, idleSpeed, testing);
    this.rotator = rotator;
    this.hoodServo = hoodServo;
    this.rotator.setRunMode(Motor.RunMode.PositionControl);
    this.rotator.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    this.ticksPerDegree = 145.1 / 360;
  }

  public Turret(DcMotorEx flywheelMotor, Motor rotator, Axon hoodServo) {
    this(flywheelMotor, rotator, hoodServo, 1500, false);
  }

  /**
   * @brief initializes the turret
   * @param horizontalAngle the angle to start the turret at
   */
  public void init(double horizontalAngle) {
    // rotator.setVelocityPIDFCoefficients(35, 2, 1, 16);
    // rotator.setPositionPIDFCoefficients(3.5);
    // rotator.setTargetPosition((int) turretDegreesToMotorDegrees(horizontalAngle));
    // rotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    // rotator.setPower(1);
    rotator.resetEncoder();
    rotator.setPositionCoefficient(0.01); // tuned (for now)
    rotator.setTargetPosition(
        (int) (turretDegreesToMotorDegrees(horizontalAngle) * ticksPerDegree));
    rotator.setPositionTolerance(turretDegreesToMotorDegrees(5) * ticksPerDegree); // TODO: tune
    hoodServo.setPosition(targetVerticalAngleDegrees);
  }

  /**
   * @brief fills the lookup table
   * @return the full lookup table
   */
  protected LookupTableItem[] fillLookupTable() {
    // TODO: fine tune lookup table
    // note: "distance" numbers *MUST* go from low to high (number, not distance)
    return new LookupTableItem[] {
      new LookupTableItem(0.1, 3500, 45),
      new LookupTableItem(0.235, 3450, 45),
      new LookupTableItem(0.27, 3250, 45),
      new LookupTableItem(0.29, 3200, 50),
      new LookupTableItem(0.4, 3000, 40),
      new LookupTableItem(0.515, 2800, 40),
      new LookupTableItem(0.635, 2700, 40),
      new LookupTableItem(0.645, 2700, 55),
      new LookupTableItem(0.67, 2700, 55),
      new LookupTableItem(0.765, 2650, 55),
      new LookupTableItem(0.85, 2600, 60),
      new LookupTableItem(0.88, 2600, 60),
      new LookupTableItem(0.99, 2600, 60),
      new LookupTableItem(1.1, 2500, 60),
      new LookupTableItem(1.4, 2500, 50),
      new LookupTableItem(1.85, 2500, 40),
      new LookupTableItem(2.17, 2450, 40),
      new LookupTableItem(2.89, 2450, 40),
      new LookupTableItem(3.85, 2400, 50),
      new LookupTableItem(4.77, 2300, 65),
      new LookupTableItem(6, 2100, 90)
    }; // preliminary values
  }

  /**
   * @brief gets if the turret is ready to shoot a ball
   * @return true if the flywheel is up to speed, the turret is at its target rotation, and the hood
   *     is in place, false otherwise
   * @note doesn't check the flywheel speed; call update() to update flywheel speed reading
   */
  @Override
  public boolean isReadyToShoot() {
    return super.isReadyToShoot() && rotator.atTargetPosition();
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
    rotator.set(0.1); // tuned (for now)
  }

  public void tune(double kP, double power) {
    rotator.setPositionCoefficient(kP);
    rotator.set(power); // TODO: tune
  }

  /**
   * @brief sets the distance to the target
   * @param distance the new distance to the target, in arbitrary units
   * @note we are actually using the percentage of the camera view occupied by the apriltag, instead
   *     of distance
   * @note the new value isn't applied until update() is called
   */
  @Override
  public void setTargetDistance(double distance) {
    setVerticalAngle(getAngleLookup(distance));
    super.setTargetDistance(distance);
  }

  /**
   * @param degrees the number of degrees from straight to move the turret
   * @brief sets the side-to-side angle of the turret in degrees
   * @note the new value isn't applied until update() is called
   */
  public void setHorizontalAngle(double degrees) {
    if (degrees > 180 + HORIZONTAL_HYSTERESIS || degrees < -180 - HORIZONTAL_HYSTERESIS) {
      // angle is wrapped to ensure the turret never turns more than ~one full rotation
      targetHorizontalAngleDegrees = AngleUnit.normalizeDegrees(degrees);

    } else {
      targetHorizontalAngleDegrees = degrees;
    }
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
   * @brief gets if the turret is at its rotational target or not
   * @return false if the rotator motor is moving to the target, true if it is at its target
   */
  public boolean isAtTarget() {
    return rotator.atTargetPosition();
  }

  /**
   * @brief sets the angle of the servo in degrees
   * @param degrees the number of degrees to move the servo to
   * @note doesn't update the turret
   * @note do not use externally except for tuning
   */
  public void setVerticalAngle(double degrees) {
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
   * @note we are actually using the percentage of the camera view occupied by the apriltag, instead
   *     of distance
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

    return map(
        distance,
        LOOKUP_TABLE[indexUnder].getDistance(),
        LOOKUP_TABLE[indexOver].getDistance(),
        LOOKUP_TABLE[indexUnder].getAngle(),
        LOOKUP_TABLE[indexOver].getAngle());
  }
}
