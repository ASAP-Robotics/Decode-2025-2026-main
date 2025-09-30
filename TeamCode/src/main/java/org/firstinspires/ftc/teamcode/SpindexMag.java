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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class SpindexMag {
  public static final int GREEN = 0; // /< spindex slot contains a green ball
  public static final int PURPLE = 1; // /< spindex slot contains a purple ball
  public static final int EMPTY = 3; // /< spindex slot doesn't contain a ball
  public static final int NULL =
      -1; // /< sort of like null (intentionally doesn't contain a useful value)
  private final ActiveIntake mag_intake; // /< the intake on the robot
  private final Servo mag_magServo; // /< the servo that rotates the divider in the mag
  private final Servo mag_liftServo; // /< the servo that lifts balls into the shooter turret
  private ColorSensor intakeColorSensor; // /< the color sensor at the intake
  private final int magSettleTime;
  private final double liftServoRestPos =
      0.0; // /< the position of the lift servo when at rest | TODO: tune
  private final double liftServoShootPos =
      0.3; // /< the position of the lift servo when shooting | TODO: tune
  private static final double[] spindexIntake = {0.0, 0.33, 0.66}; // TODO: tune
  private static final double[] spindexShoot = {0.33, 0.66, 0.00}; // TODO: tune
  private int[] spindexColor = {EMPTY, EMPTY, EMPTY};
  private int mag_intakeIndex;
  private int mag_shootIndex;

  public SpindexMag(
      ActiveIntake intake, Servo spinServo, Servo liftServo, ColorSensor colorSensor) {
    mag_intake = intake;
    mag_magServo = spinServo;
    mag_liftServo = liftServo;
    intakeColorSensor = colorSensor;
    magSettleTime = 200;
  }

  /**
   * @brief updates everything to do with the mag
   * @note call each loop
   */
  public void update() {
    updateIntake();
  }

  /**
   * @brief updates everything to do with intaking balls
   */
  private void updateIntake() {
    if (mag_intake.intaking) { // if the intake is trying to intake a ball
      int intakeColor = getIntakeColor(); // get the color of ball (if any) in the intake position
      if (intakeColor != EMPTY) { // if a ball is in the intake position
        mag_intake.stop(); // stop the intake
        spindexColor[mag_intakeIndex] = intakeColor; // record the color of the ball taken in
      }
    } else if (mag_intake.ejecting) {
      int intakeColor = getIntakeColor(); // get the color of ball (if any) in the intake position
      if (intakeColor == EMPTY) { // if there isn't a ball in the intake position
        mag_intake.stop(); // stop the intake
      }
    }
  }

  /**
   * @brief intakes a ball to the first empty intake index in the spindex
   * @return true if a ball was taken in, false if the mag was full
   */
  public boolean intakeBall() {
    int index = NULL;
    for (int i = 0; i < spindexColor.length; i++) {
      if (i == EMPTY) index = i;
    }

    if (index == NULL) return false; // if there are no empty slots, return false

    return intakeBallIndex(mag_intakeIndex); // intake ball to index currently at the intake
  }

  /**
   * @brief intakes a ball at a specified index in the spindex
   * @param index the index in the spindex to put the ball in
   * @return true if a ball was taken in, false if a ball was already in the given index
   */
  private boolean intakeBallIndex(int index) {
    if (getColorIndex(index) != EMPTY) return false;

    mag_liftServo.setPosition(liftServoRestPos); // lower lifter out of the way
    moveSpindexIntake(index); // move spindex to correct position
    mag_intake.intake(); // intake a ball
    spindexColor[index] = getIntakeColor(); // set color of ball in spindex index

    return true;
  }

  /**
   * @brief shoots a requested color, if possible
   * @param color the color to be shot
   * @return true if ball shot, false if no balls of requested color are in mag
   */
  public boolean shootColor(int color) {
    int index = getColorIndex(color); // find index in the spindex of requested color
    if (index == NULL) return false;
    shootIndex(index);
    return true;
  }

  /**
   * @brief shoots a ball from a given index in the spindex
   * @param index the ball from the spindex to be shot
   * @note placeholder, should be updated; assumes servos move instantly
   */
  private void shootIndex(int index) {
    // TODO: fix logic; servos don't move instantly
    // TODO: add flywheel control stuff here
    moveSpindexShoot(index); // move spindex to correct position
    mag_liftServo.setPosition(liftServoShootPos); // lift ball into flywheel
    // TODO: add some sort of delay something here, but not blocking
    mag_liftServo.setPosition(liftServoRestPos); // reset lifter
  }

  /**
   * @brief moves the specified spindex index to its intake position
   * @param index the spindex index to move to the intake position
   */
  private void moveSpindexIntake(int index) {
    mag_magServo.setPosition(spindexIntake[index]);
    mag_intakeIndex = index; // set new intake index
    mag_shootIndex = NULL; // spindex isn't at a shooting index
  }

  /**
   * @brief moves the specified spindex index to its shooting position
   * @param index the spindex index to move to the shooting position
   */
  private void moveSpindexShoot(int index) {
    mag_magServo.setPosition(spindexShoot[index]);
    mag_shootIndex = index; // set new shooting index
    mag_intakeIndex = NULL; // spindex isn't at an intake index
  }

  /**
   * @brief gets the color of ball in the intake position
   * @return the color of ball in the intake
   */
  private int getIntakeColor() {
    return 0; // placeholder; TODO: change
  }

  /**
   * @brief finds an index in the spindex containing a given color
   * @param color the color wanted
   * @return the index in the spindex where that color is located, or -1 if that color isn't in the
   *     mag
   */
  private int getColorIndex(int color) {
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
