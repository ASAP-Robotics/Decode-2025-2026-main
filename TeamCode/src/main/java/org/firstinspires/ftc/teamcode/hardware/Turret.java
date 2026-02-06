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

import static org.firstinspires.ftc.teamcode.utils.MathUtils.map;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.hardware.sensors.ElcAbsEncoderAnalog;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.Follower;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.jetbrains.annotations.TestOnly;

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

  private static final double HORIZONTAL_WRAP_CENTER_DEGREES = -90.0;
  // amount position has to change to actually set servo
  private static final double SERVO_UPDATE_TOLERANCE = 0.5; // degrees
  // amount power has to change by to actually set (rotator) motor
  private static final double MOTOR_UPDATE_TOLERANCE = 0.01; // % power
  // number of teeth on the gear attached to the turret
  private static final double TURRET_GEAR_TEETH = 120;
  // number of teeth on the gear attached to the motor
  private static final double MOTOR_GEAR_TEETH = 24;
  // amount horizontal angle can go over 180 or under -180 degrees before wrapping
  private static final double HORIZONTAL_HYSTERESIS = 10;
  private static final double HORIZONTAL_TOLERANCE = 3; // degrees
  protected Follower angleSimulation; // simulation of the horizontal angle of the turret
  protected SystemStatus turretStatus = SystemStatus.NOMINAL;
  private final ElcAbsEncoderAnalog encoder;
  private final Motor rotator;
  private final PIDController rotatorController;
  // ^ PID controller for horizontal rotation of turret, uses motor degrees as units
  private final Axon hoodServo;
  private double targetHorizontalAngleDegrees = 0;
  private double horizontalAngleOffsetDegrees = 0;
  // target angle for servo moving flap
  private double targetVerticalAngleDegrees = 50;
  private double testingVerticalAngleDegrees = 50;
  private double lastSetVerticalAngleDegrees = Double.NEGATIVE_INFINITY;
  private double currentRotatorPower = 0;
  private boolean rotationEnabled = true; // if turret can move side to side

  public Turret(
      DcMotorEx flywheelMotor,
      Motor rotator,
      ElcAbsEncoderAnalog encoder,
      Axon hoodServo,
      double idleSpeed) {
    super(flywheelMotor, idleSpeed);
    this.rotator = rotator;
    this.encoder = encoder;
    this.hoodServo = hoodServo;
    this.rotator.setInverted(true);
    this.rotator.setRunMode(Motor.RunMode.RawPower);
    this.rotator.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    this.rotatorController = new PIDController(0.015, 0.03, 0.0007);
    rotatorController.setTolerance(turretDegreesToMotorDegrees(1));
    this.encoder.setInverted(true);
    angleSimulation = new Follower(0, 0, 1, 60); // tune 60
  }

  public Turret(
      DcMotorEx flywheelMotor, Motor rotator, ElcAbsEncoderAnalog encoder, Axon hoodServo) {
    this(flywheelMotor, rotator, encoder, hoodServo, 1500);
  }

  /**
   * @brief initializes the turret
   * @param horizontalAngle the angle to start the turret at
   * @note if angle is zero, the turret will not move
   */
  public void init(double horizontalAngle) {
    setHorizontalAngle(horizontalAngle);
    rotatorController.setSetPoint(turretDegreesToMotorDegrees(targetHorizontalAngleDegrees));
    rotator.set(0);
    if (horizontalAngle == 0) rotationEnabled = false;
    if (rotationEnabled) {
      rotator.stopAndResetEncoder();
      rotator.setRunMode(Motor.RunMode.RawPower);
      calculateTurretOffset();
      hoodServo.setPosition(targetVerticalAngleDegrees);
      lastSetVerticalAngleDegrees = targetVerticalAngleDegrees;
    }
  }

  /**
   * @brief fills the lookup table
   * @return the full lookup table
   */
  protected LookupTableItem[] fillLookupTable() {
    // note: "distance" numbers *MUST* go from low to high
    return new LookupTableItem[] {
      // Extrapolated 0 point (based on 28.2 values)
      new LookupTableItem(0, 1900, 160),

      // Tuned Data (Sorted Low to High)
      new LookupTableItem(28.2, 1900, 160),
      new LookupTableItem(34.2, 1900, 60),
      new LookupTableItem(42.9, 2040, 50),
      new LookupTableItem(46.7, 2160, 42),
      new LookupTableItem(60.1, 2350, 33),
      new LookupTableItem(75.6, 2450, 31),
      new LookupTableItem(93.3, 2600, 30),
      new LookupTableItem(107.1, 2800, 20),
      new LookupTableItem(118.4, 2950, 15),
      new LookupTableItem(133.8, 3100, 10),
      new LookupTableItem(164.6, 3300, 10),

      // Extrapolated "infinite" point (based on 164.6 values)
      new LookupTableItem(Double.POSITIVE_INFINITY, 3300, 10)
    };
  }

  @Override
  public SystemReport getStatus() {
    SystemReport report = super.getStatus();
    SystemReport toReturn = new SystemReport(SystemStatus.NOMINAL);
    if (report.status.severity > toReturn.status.severity) {
      String message = "⁉️Unknown (Flywheel)";
      switch (report.status) {
        case FALLBACK:
          message = "🟨Fallback (Flywheel)";
          break;

        case INOPERABLE:
          message = "🟥Broken (Flywheel)";
          break;
      }

      toReturn = new SystemReport(report.status, message);
    }

    if (turretStatus.severity > toReturn.status.severity) {
      String message = "⁉️Unknown (Turret)";
      switch (report.status) {
        case FALLBACK:
          message = "🟨Fallback (Turret)";
          break;

        case INOPERABLE:
          message = "🟥Broken (Turret)";
          break;
      }

      toReturn = new SystemReport(turretStatus, message);
    }

    return toReturn;
  }

  /**
   * @brief gets if the turret is ready to shoot a ball
   * @return true if the flywheel is up to speed, the turret is at its target rotation, and the hood
   *     is in place, false otherwise
   * @note doesn't check the flywheel speed; call update() to update flywheel speed reading
   */
  @Override
  public boolean isReadyToShoot() {
    return super.isReadyToShoot() && (isRotatorAtTarget() || angleSimulation.isAtTarget());
  }

  /**
   * @brief gets if the turret is at its target horizontal angle
   * @return true if the turret is at target, false otherwise
   */
  protected boolean isRotatorAtTarget() {
    double angle = getHorizontalAngleDegrees();
    double targetAngle = getTargetHorizontalAngleDegrees();
    return angle + HORIZONTAL_TOLERANCE > targetAngle && angle - HORIZONTAL_TOLERANCE < targetAngle;
  }

  /**
   * @brief to be called once, when the program is started
   */
  public void start() {
    if (!rotationEnabled) {
      rotator.stopAndResetEncoder();
      rotator.setRunMode(Motor.RunMode.RawPower);
      calculateTurretOffset();
    }
    rotationEnabled = true;
  }

  /**
   * @brief updates the turret
   * @note call every loop
   */
  @Override
  public void update() {
    super.update();

    double targetServoDegrees = testing ? testingVerticalAngleDegrees : targetVerticalAngleDegrees;
    if (Math.abs(targetServoDegrees - lastSetVerticalAngleDegrees) > SERVO_UPDATE_TOLERANCE) {
      hoodServo.setPosition(targetServoDegrees);
      lastSetVerticalAngleDegrees = targetServoDegrees;
    }

    double motorDegrees =
        turretDegreesToMotorDegrees(targetHorizontalAngleDegrees + horizontalAngleOffsetDegrees);
    rotatorController.setSetPoint(motorDegrees);
    double targetRotatorPower =
        rotationEnabled ? rotatorController.calculate(getRotatorDegrees()) : 0;
    if (Math.abs(targetRotatorPower - currentRotatorPower) > MOTOR_UPDATE_TOLERANCE) {
      rotator.set(targetRotatorPower);
      currentRotatorPower = targetRotatorPower;
    }

    turretStatus =
        rotationEnabled && !isRotatorAtTarget() && angleSimulation.isAtTarget()
            ? SystemStatus.FALLBACK
            : SystemStatus.NOMINAL;
  }

  /**
   * @brief used to tune the PID for the horizontal rotation of the turret
   * @param kP the proportional constant for the rotator
   * @param kI the integral constant for the rotator
   * @param kD the derivative constant for the rotator
   */
  @TestOnly
  public void tuneHorizontalPID(double kP, double kI, double kD) {
    rotatorController.setPID(kP, kI, kD);
  }

  /**
   * @brief used for tuning the lookup table, provides manual control of the turret
   * @param rpm the rpm to spin the flywheel at
   * @param angle the angle to move the hood servo to
   */
  @TestOnly
  public void tuneShooting(double rpm, double angle) {
    overrideRpm(rpm);
    overrideVerticalAngle(angle);
  }

  /**
   * @brief used for tuning, overrides the vertical angle
   * @param angleDegrees the angle to set the flap at
   */
  protected void overrideVerticalAngle(double angleDegrees) {
    testing = true;
    testingVerticalAngleDegrees = angleDegrees;
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
    if (Double.isNaN(degrees)) return;

    double min = HORIZONTAL_WRAP_CENTER_DEGREES - 180 - HORIZONTAL_HYSTERESIS;
    double max = HORIZONTAL_WRAP_CENTER_DEGREES + 180 + HORIZONTAL_HYSTERESIS;

    if (degrees > max || degrees < min) {
      // angle is wrapped to ensure the turret never turns more than ~one full rotation
      targetHorizontalAngleDegrees =
          MathUtils.normalizeAround(degrees, HORIZONTAL_WRAP_CENTER_DEGREES);

    } else {
      targetHorizontalAngleDegrees = degrees;
    }

    angleSimulation.setTarget(targetHorizontalAngleDegrees);
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
   * @note returned value is adjusted using the horizontal angle offset
   */
  public double getHorizontalAngleDegrees() {
    return motorDegreesToTurretDegrees(getRotatorDegrees()) - horizontalAngleOffsetDegrees;
  }

  /** Calculates the angle offset for the turret */
  private void calculateTurretOffset() {
    horizontalAngleOffsetDegrees =
        motorDegreesToTurretDegrees(encoder.getAngleNormalized() + getRotatorDegrees()) + 4.5;
  }

  /**
   * @brief gets the current position of the rotator motor in degrees
   * @return the position of the rotator motor, according to the encoder, in degrees
   * @note this value is not adjusted using the horizontal angle offset
   */
  protected double getRotatorDegrees() {
    return -rotator.getCurrentPosition() / 4000.0 * 360.0;
  }

  /**
   * Forces a re-calculation of the turret offset using the absolute encoder
   *
   * @note this is here mainly as a driver backup; even as a backup use with caution, can easily
   *     make a bad situation worse
   */
  public void syncEncoder() {
    calculateTurretOffset();
  }

  /**
   * @brief changes the horizontal angle offset for the turret, to account for belt slippage
   * @param deltaDegrees the amount to change the horizontal angle offset by, in degrees
   */
  public void changeHorizontalAngleOffsetDegrees(double deltaDegrees) {
    horizontalAngleOffsetDegrees += deltaDegrees;
  }

  /**
   * @brief sets the horizontal angle offset for the turret, to account for belt slippage
   * @param offsetDegrees the angle to offset the turret angle by, in degrees
   */
  public void setHorizontalAngleOffsetDegrees(double offsetDegrees) {
    horizontalAngleOffsetDegrees = offsetDegrees;
  }

  /**
   * @brief gets the horizontal angle offset for the turret, used to account for belt slippage
   * @return the horizontal angle offset, in degrees
   */
  public double getHorizontalAngleOffsetDegrees() {
    return horizontalAngleOffsetDegrees;
  }

  /**
   * @brief gets if the turret is at its rotational target or not
   * @return false if the rotator motor is moving to the target, true if it is at its target
   */
  public boolean isAtTarget() {
    double angle = getHorizontalAngleDegrees();
    double target = getTargetHorizontalAngleDegrees();
    return angle - 5 < target && angle + 5 > target;
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

      return map(
          distance,
          LOOKUP_TABLE[indexUnder].getDistance(),
          LOOKUP_TABLE[indexOver].getDistance(),
          LOOKUP_TABLE[indexUnder].getAngle(),
          LOOKUP_TABLE[indexOver].getAngle());
    } catch (Exception e) {
      return 50; // placeholder angle if distance outside of table
    }
  }
}
