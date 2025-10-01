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

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SpindexMag {
  public static enum BallColor {
    GREEN,
    PURPLE,
    EMPTY,
    UNKNOWN
  }
  public static final int NULL =
      -1; // /< sort of like null (intentionally doesn't contain a useful value)
  private final ActiveIntake intake; // /< the intake on the robot
  private final Flywheel flywheel;
  private final Servo spinServo; // /< the servo that rotates the divider in the mag
  private final Servo liftServo; // /< the servo that lifts balls into the shooter turret
  private final ColorSensor colorSensor; // /< the color sensor at the intake
  private final DistanceSensor distanceSensor; // /< the distance sensor at the intake (built into color sensor?)
  private final double liftServoRestPos =
      0.0; // /< the position of the lift servo when at rest | TODO: tune
  private final double liftServoShootPos =
      0.3; // /< the position of the lift servo when shooting | TODO: tune
  private static final double[] spindexIntake = {0.0, 0.33, 0.66}; // TODO: tune
  private static final double[] spindexShoot = {0.33, 0.66, 0.00}; // TODO: tune
  private BallColor[] spindexColor = {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
  private int mag_intakeIndex;
  private int mag_shootIndex;

  public SpindexMag(
      ActiveIntake intake, Flywheel flywheel, Servo spinServo, Servo liftServo, ColorSensor colorSensor, DistanceSensor distanceSensor) {
    this.intake = intake;
    this.flywheel = flywheel;
    this.spinServo = spinServo;
    this.liftServo = liftServo;
    this.colorSensor = colorSensor;
    this.distanceSensor = distanceSensor;
    this.colorSensor.enableLed(true); // turn on color sensor LED
  }

  /**
   * @brief updates everything to do with the mag
   * @note call each loop
   */
  public void update() {
    updateShooting();
    updateIntake();
  }

  /**
   * @brief updates everything to do with shooting balls out of the turret
   */
  private void updateShooting() {
    if (flywheel.isEnabled() && flywheel.isActive()) {
      if (flywheel.containsBall) {
        if (flywheel.flywheel_shotTimer.isFinished()) {
          flywheel.containsBall = false; // the ball should be out of the flywheel
          flywheel.idle(); // set the flywheel to slow down to idle speeds
        }
      } else {
        if (flywheel.isUpToSpeed()) {
          liftServo.setPosition(liftServoShootPos); // lift ball into flywheel
          flywheel.containsBall = true;
        }
      }
    }
  }

  /**
   * @brief updates everything to do with the intake
   */
  private void updateIntake() {
    if (intake.intaking) { // if the intake is trying to intake a ball
      BallColor intakeColor = getIntakeColor(); // get the color of ball (if any) in the intake position
      if (intakeColor != BallColor.EMPTY) { // if a ball is in the intake position
        intake.stop(); // stop the intake
        spindexColor[mag_intakeIndex] = intakeColor; // record the color of the ball taken in
      }
    } else if (intake.ejecting) {
      BallColor intakeColor = getIntakeColor(); // get the color of ball (if any) in the intake position
      if (intakeColor == BallColor.EMPTY) { // if there isn't a ball in the intake position
        intake.stop(); // stop the intake
      }
    }
  }

  /**
   * @brief intakes a ball to the first empty intake index in the spindex
   * @return true if a ball was taken in, false if the mag was full or intake is busy
   */
  public boolean intakeBall() {
    int index = NULL;
    for (int i = 0; i < spindexColor.length; i++) {
      if (spindexColor[i] == BallColor.EMPTY) index = i;
    }

    if (index == NULL) return false; // if there are no empty slots, return false

    return intakeBallIndex(mag_intakeIndex); // intake ball to index currently at the intake
  }

  /**
   * @brief intakes a ball at a specified index in the spindex
   * @param index the index in the spindex to put the ball in
   * @return true if given index was empty, false if a ball was already in the given index or intake is busy
   */
  private boolean intakeBallIndex(int index) {
    if ((spindexColor[index] != BallColor.EMPTY) || intake.busy) return false; // return false if given index contains a ball, or intake is busy

    liftServo.setPosition(liftServoRestPos); // lower lifter out of the way
    moveSpindexIntake(index); // move spindex to correct position
    intake.intake(); // start the intake spinning

    return true;
  }

  /**
   * @brief shoots a requested color, if possible
   * @param color the color to be shot
   * @return true if ball shot, false if no balls of requested color are in mag
   */
  public boolean shootColor(BallColor color) {
    int index = getColorIndex(color); // find index in the spindex of requested color
    if (index == NULL) return false;
    shootIndex(index);
    return true;
  }

  /**
   * @brief starts shooting a ball from a given index in the spindex
   * @param index the ball from the spindex to be shot
   * @note this is a work in progress
   */
  private void shootIndex(int index) {
    // NOTE: assumes that the servos will move in the time it takes the flywheel to spin up
    flywheel.activate(); // spin flywheel up to speed
    moveSpindexShoot(index); // move spindex to correct position
  }

  /**
   * @brief moves the specified spindex index to its intake position
   * @param index the spindex index to move to the intake position
   */
  private void moveSpindexIntake(int index) {
    spinServo.setPosition(spindexIntake[index]);
    mag_intakeIndex = index; // set new intake index
    mag_shootIndex = NULL; // spindex isn't at a shooting index
  }

  /**
   * @brief moves the specified spindex index to its shooting position
   * @param index the spindex index to move to the shooting position
   */
  private void moveSpindexShoot(int index) {
    spinServo.setPosition(spindexShoot[index]);
    mag_shootIndex = index; // set new shooting index
    mag_intakeIndex = NULL; // spindex isn't at an intake index
  }

  /**
   * @brief gets the color of ball in the intake position
   * @return the color of ball in the intake
   */
  private BallColor getIntakeColor() {
    BallColor toReturn = BallColor.EMPTY;

    int red = colorSensor.red();
    int green = colorSensor.green();
    int blue = colorSensor.blue();
    float[] hsv = new float[3];
    Color.RGBToHSV(red * 8, green * 8, blue * 8, hsv);
    float h = hsv[0];
    float s = hsv[1];
    float v = hsv[2];

    if (s > 0.6 && v > 40 && h >= 150 && h <= 170) { // green
      toReturn = BallColor.GREEN;
    } else if (s > 0.3 && v > 40 && h >= 220 && h <= 240) { // purple
      toReturn = BallColor.PURPLE;
    } else { // color can't be determined
      double distance = distanceSensor.getDistance(DistanceUnit.INCH);
      if (distance <= 2.0) { // if a ball is in the intake
        toReturn = BallColor.UNKNOWN;
      }
    }

    return toReturn;
  }

  /**
   * @brief finds an index in the spindex containing a given color
   * @param color the color wanted
   * @return the index in the spindex where that color is located, or -1 if that color isn't in the
   *     mag
   */
  private int getColorIndex(BallColor color) {
    int toReturn = NULL;

    for (int i = 0; i < spindexColor.length; i++) { // for each spindex slot
      if (spindexColor[i] == color) { // if the slot contains the correct color
        toReturn = i; // record slot index
        break; // don't keep looking for a slot
      }
    }

    return toReturn;
  }
}
