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
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumWheelBase {
  private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
  private final double sensitivityCurve; // exponent used to scale input throttle values
  private double rotation = 0; // number of degrees the robot is rotated relative to field forward
  private double rawThrottleX = 0, rawThrottleY = 0, rawThrottleZ = 0; // raw throttle values
  private double throttleX = 0, throttleY = 0, throttleZ = 0; // processed (robot) throttle values
  private double minAccelerationTime; // the time in seconds to go from 0 to full speed
  private boolean fieldCentric = false; // whether or not to use field-centric control
  private ElapsedTime lastUpdateTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

  public MecanumWheelBase(
      DcMotorEx frontLeft,
      DcMotorEx frontRight,
      DcMotorEx backLeft,
      DcMotorEx backRight,
      double accelerationTime,
      double sensitivityCurve) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;
    this.minAccelerationTime = accelerationTime;
    this.sensitivityCurve = sensitivityCurve;
    // set motors to power-based control
    this.frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    this.frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    this.backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    this.backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    // set motors to float when set to zero power
    this.frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    this.frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    this.backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    this.backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    // reverse motors on left side of the robot
    this.frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
    this.backLeft.setDirection(DcMotorEx.Direction.REVERSE);
  }

  public MecanumWheelBase(
      DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
    this(frontLeft, frontRight, backLeft, backRight, 1, 2);
  }

  /**
   * @brief stops all powered wheel movement (think E-stop)
   * @note call only when the "Stop" button is pressed; otherwise things will break
   */
  public void stop() {
    // set motors to simple power-level control
    frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    // set motors to brake when set to 0% power
    frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    // set motor powers to 0%
    frontLeft.setPower(0);
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);
  }

  /**
   * @brief sets weather or not field-centric control will be used
   * @param fieldCentric true for field-centric control, false for robot-centric control
   */
  public void setFieldCentric(boolean fieldCentric) {
    this.fieldCentric = fieldCentric;
  }

  /**
   * @brief sets the minimum time that the robot can go from 0% to 100% throttle
   * @param seconds the time in seconds to go from 0 to full speed
   */
  public void setAccelerationTime(double seconds) {
    minAccelerationTime = seconds;
  }

  /**
   * @brief sets the rotation of the robot relative to the field (for field-centric control)
   * @param degrees the number of degrees the robot is rotated relative to the field
   */
  public void setRotation(double degrees) {
    rotation = degrees;
  }

  /**
   * @brief sets all throttle values controlling the wheel base
   * @param x the throttle controlling the x-axis speed
   * @param y the throttle controlling the y-axis speed
   * @param z the throttle controlling turning speed
   * @note inputs must be between 1 and -1, values below 0 reverse direction. Does not set motors;
   *     call update() to apply
   */
  public void setThrottle(double x, double y, double z) {
    setThrottleX(x, false);
    setThrottleY(y, false);
    setThrottleZ(z, true);
  }

  /**
   * @brief sets the throttle value controlling the x-axis motion of the wheel base
   * @param x the throttle controlling the x-axis speed
   * @note input must be between 1 and -1, values below 0 reverse direction
   */
  public void setThrottleX(double x) {
    setThrottleX(x, true);
  }

  /**
   * @brief sets the throttle value controlling the x-axis motion of the wheel base
   * @param x the throttle controlling the x-axis speed
   * @param update weather or not the motors should be updated with the new value
   * @note input must be between 1 and -1, values below 0 reverse direction
   */
  private void setThrottleX(double x, boolean update) {
    rawThrottleX = x;
    if (update) update();
  }

  /**
   * @brief sets the throttle value controlling the y-axis motion of the wheel base
   * @param y the throttle controlling the y-axis speed
   * @note input must be between 1 and -1, values below 0 reverse direction
   */
  public void setThrottleY(double y) {
    setThrottleY(y, true);
  }

  /**
   * @brief sets the throttle value controlling the y-axis motion of the wheel base
   * @param y the throttle controlling the y-axis speed
   * @param update weather or not the motors should be updated with the new value
   * @note input must be between 1 and -1, values below 0 reverse direction
   */
  public void setThrottleY(double y, boolean update) {
    rawThrottleY = y;
    if (update) update();
  }

  /**
   * @brief sets the throttle value controlling the turning motion of the wheel base
   * @param z the throttle controlling the turning speed
   * @note input must be between 1 and -1, values below 0 reverse direction
   */
  public void setThrottleZ(double z) {
    setThrottleZ(z, true);
  }

  /**
   * @brief sets the throttle value controlling the turning motion of the wheel base
   * @param z the throttle controlling the turning speed
   * @param update weather or not the motors should be updated with the new value
   * @note input must be between 1 and -1, values below 0 reverse direction
   */
  public void setThrottleZ(double z, boolean update) {
    rawThrottleZ = z;
    if (update) update();
  }

  /**
   * @brief adjusts throttle inputs from 1 to -1 to fit a sensitivity curve
   * @param input the throttle input, from 1 to -1
   * @return the adjusted throttle output, from 1 to -1
   * @note if passed an input less than -1 or greater than 1, 0 will be returned
   */
  private double scaleThrottle(double input) {
    return scaleThrottle(input, 0, 0, false);
  }

  /**
   * @brief adjusts throttle inputs from 1 to -1 to fit a sensitivity curve and limit acceleration
   * @param input the throttle input, from 1 to -1
   * @param lastOutput the value used to calculate maximum output increase
   * @param maxIncrease the maximum amount the output is allowed to increase by
   * @return the adjusted throttle output, from 1 to -1
   * @note if passed an input less than -1 or greater than 1, 0 will be returned
   */
  private double scaleThrottle(double input, double lastOutput, double maxIncrease) {
    return scaleThrottle(input, lastOutput, maxIncrease, true);
  }

  /**
   * @brief adjusts throttle inputs from 1 to -1 to fit a sensitivity curve
   * @param input the throttle input, from 1 to -1
   * @param lastOutput the value used to calculate maximum output increase
   * @param maxIncrease the maximum amount the output is allowed to increase by
   * @param limitAcceleration if the output will be clamped to lastOutput + maxIncrease
   * @return the adjusted throttle output, from 1 to -1
   * @note if passed an input less than -1 or greater than 1, 0 will be returned
   */
  private double scaleThrottle(double input,
                               double lastOutput,
                               double maxIncrease,
                               boolean limitAcceleration) {
    double filteredInput = Math.min(Math.abs(input), 1); // clamp absolute value below 1
    double scaledValue = Math.pow(filteredInput, sensitivityCurve); // fit to curve
    if (limitAcceleration) // if acceleration is limited
      scaledValue = Math.min(Math.abs(lastOutput) + maxIncrease, scaledValue); // limit acceleration

    if ((input > 0) && (input <= 1)) { // input is positive and in bounds
      return scaledValue;

    } else if ((input < 0) && (input >= -1)) { // input is negative and in bounds
      return -scaledValue;

    } else { // input is 0 or out of bounds
      return 0;
    }
  }

  /**
   * @brief rotates the vector of a pair of throttle inputs by the given number of degrees
   * @param degrees the number of degrees to rotate the throttle by
   * @param x the input x throttle value
   * @param y the input y throttle value
   * @return an array with the rotated throttle values in the format [x, y]
   */
  private double[] rotateThrottle(double degrees, double x, double y) {
    double radians = Math.toRadians(degrees);

    // rotate throttle values
    double rotatedX = x * Math.cos(radians) - y * Math.sin(radians);
    double rotatedY = x * Math.sin(radians) + y * Math.cos(radians);

    return new double[] {rotatedX, rotatedY};
  }

  /**
   * @brief sets motor target speeds according to throttle values
   * @note this override uses robot-centric control. Call update(true) for field-centric control
   */
  public void update() {
    update(fieldCentric);
  }

  /**
   * @brief sets motor target speeds according to throttle values
   * @param fieldCentric driving will be field-centric if true, robot-centric if false
   */
  public void update(boolean fieldCentric) {
    double timeSinceLastUpdate = lastUpdateTimer.seconds(); // get time since last update
    lastUpdateTimer.reset(); // reset time since last update
    double maxIncrease = timeSinceLastUpdate / minAccelerationTime; // find max throttle increase
    // scale raw throttle values to match sensitivity curve
    // x throttle is increased to account for imperfect strafing
    // x will be able to accelerate slightly faster than y and z due to this
    throttleX = scaleThrottle(rawThrottleX, throttleX, maxIncrease); // get processed x throttle
    throttleX = Math.min(1, Math.max(-1, throttleX * 1.1)); // adjust for imperfect strafing
    throttleY = scaleThrottle(rawThrottleY, throttleY, maxIncrease); // get processed y throttle
    throttleZ = scaleThrottle(rawThrottleZ, throttleZ, maxIncrease); // get processed z throttle

    if (fieldCentric) { // rotate throttle values if field centric
      double[] rotated = rotateThrottle(rotation, throttleX, throttleY); // rotate throttle
      throttleX = rotated[0];
      throttleY = rotated[1];
    }

    // find motor power ratios
    double fl = throttleY + throttleX + throttleZ;
    double fr = throttleY - throttleX - throttleZ;
    double bl = throttleY - throttleX + throttleZ;
    double br = throttleY + throttleX - throttleZ;

    // find maximum motor power (or 100% if none exceed 100%)
    double maxTargetSpeed =
        Math.max(
            1.0,
            Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

    // adjust motor powers so none exceed 100% power
    fl /= maxTargetSpeed;
    fr /= maxTargetSpeed;
    bl /= maxTargetSpeed;
    br /= maxTargetSpeed;

    // set motor target powers
    frontLeft.setPower(fl);
    frontRight.setPower(fr);
    backLeft.setPower(bl);
    backRight.setPower(br);
  }
}
