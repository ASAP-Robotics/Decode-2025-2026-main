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
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

public class SpindexMag {
  public static enum BallColor {
    GREEN,
    PURPLE,
    EMPTY,
    UNKNOWN
  }

  public static enum BallSequence {
    GPP(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE),
    PGP(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE),
    PPG(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN);

    private final BallColor[] ballColors;

    BallSequence(BallColor... ballColors) {
      this.ballColors = ballColors;
    }

    public BallColor[] getBallColors() {
      return ballColors.clone();
    }
  }

  public static final int NULL =
      -1; // /< sort of like null (intentionally doesn't contain a useful value)
  private final ActiveIntake intake; // /< the intake on the robot
  private final Flywheel flywheel;
  private final Servo spinServo; // /< the servo that rotates the divider in the mag
  private final Servo liftServo; // /< the servo that lifts balls into the shooter turret
  private final ColorSensor colorSensor; // /< the color sensor at the intake
  private final DistanceSensor
      distanceSensor; // /< the distance sensor at the intake (built into color sensor?)
  private final double liftServoRestPos =
      0.0; // /< the position of the lift servo when at rest | TODO: tune
  private final double liftServoShootPos =
      0.3; // /< the position of the lift servo when shooting | TODO: tune
  private static final double[] spindexIntake = {0.0, 0.33, 0.66}; // TODO: tune
  private static final double[] spindexShoot = {0.33, 0.66, 0.00}; // TODO: tune
  private final BallColor[] spindexColor = {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
  private int mag_intakeIndex;
  private int mag_shootIndex;
  private boolean fillingMag = false; // /< if the mag is being filled
  private boolean shootingSequence = false; // /< if a sequence is being shot out of the turret
  private BallSequence ballSequence; // /< the sequence being shot
  private int sequenceIndex = 0; // /< the index of ball in the sequence that is being shot
  private int purplesNeeded = 0; // the number of purples needed to fill the mag
  private int greensNeeded = 0; // the number of greens needed to fill the mag
  private final org.firstinspires.ftc.teamcode.utils.SimpleTimer liftServoTimer =
      new SimpleTimer(0.5); // /< timer for lifting ball into flywheel
  private final org.firstinspires.ftc.teamcode.utils.SimpleTimer spinServoTimer =
      new SimpleTimer(0.75); // /< timer for moving spindex
  private final Telemetry telemetry;

  public SpindexMag(
      ActiveIntake intake,
      Flywheel flywheel,
      Servo spinServo,
      Servo liftServo,
      ColorSensor colorSensor,
      DistanceSensor distanceSensor,
      Telemetry telemetry) {
    this.intake = intake;
    this.flywheel = flywheel;
    this.spinServo = spinServo;
    this.liftServo = liftServo;
    this.colorSensor = colorSensor;
    this.distanceSensor = distanceSensor;
    this.colorSensor.enableLed(true); // turn on color sensor LED
    this.telemetry = telemetry;
  }

  /**
   * @brief updates everything to do with the mag
   * @note call each loop
   */
  public void update() {
    updateShooting();
    updateIntake();
    flywheel.update();
    telemetry.addData("Mag", Arrays.toString(spindexColor));
    telemetry.addData("Shooting", shootingSequence);
    telemetry.addData("Filling", fillingMag);
  }

  /**
   * @brief updates everything to do with shooting balls out of the turret
   */
  private void updateShooting() {
    if (shootingSequence) { // if we are shooting a sequence (rapid fire)
      BallColor[] sequence = ballSequence.getBallColors();
      BallColor shootingColor = sequence[sequenceIndex];
      telemetry.addData("Shooting color", shootingColor);
      telemetry.addData("Shooting index", sequenceIndex);
      if (flywheel.getContainsBall()) { // if there is a ball in the flywheel
        if (liftServoTimer.isFinished()) {
          liftServo.setPosition(liftServoRestPos);
        }
        if (flywheel.shotTimer.isFinished()) { // if the ball entered the flywheel long enough ago
          flywheel.setContainsBall(false); // there is not a ball in the flywheel
          spindexColor[mag_shootIndex] = BallColor.EMPTY; // spindex slot is now empty
          if (sequenceIndex < sequence.length) { // if the sequence isn't done
            sequenceIndex++; // move on to the next ball in the sequence

          } else { // if the sequence is done
            shootingSequence = false; // we are no longer shooting a sequence
            flywheel.idle(); // let flywheel slow down to idle speed
          }
        }

      } else { // if there isn't a ball in the flywheel
        int shootingColorIndex = getColorIndex(shootingColor);
        if ((!spinServoTimer.isRunning())
            && (mag_shootIndex != shootingColorIndex)) { // if the spindex position hasn't been set
          moveSpindexShoot(shootingColorIndex); // move spindex to correct location

        } else if (flywheel.isUpToSpeed()
            && spinServoTimer
                .isFinished()) { // if the flywheel is up to speed and the spindex is done moving
          liftServo.setPosition(liftServoShootPos); // lift ball into flywheel
          liftServoTimer.start();
          flywheel.setContainsBall(true); // flywheel now has a ball in it
          flywheel.shotTimer.start();
        }
      }

    } else if (flywheel.isEnabled() && flywheel.isActive()) { // if we are just shooting one ball
      if (flywheel.getContainsBall()) {
        if (flywheel.shotTimer.isFinished()) {
          flywheel.setContainsBall(false); // the ball should be out of the flywheel
          flywheel.idle(); // set the flywheel to slow down to idle speeds
        }

      } else {
        if (flywheel.isUpToSpeed()) {
          liftServo.setPosition(liftServoShootPos); // lift ball into flywheel
          flywheel.setContainsBall(true);
        }
      }
    }
  }

  /**
   * @brief updates everything to do with the intake
   */
  private void updateIntake() {
    if (fillingMag) { // if we are filling the magazine
      BallColor intakeColor =
          getIntakeColor(); // get the color of ball (if any) in the intake position
      if (intake.isIntaking()) {
        if (intakeColor == BallColor.PURPLE) { // if a purple is in the intake
          if (purplesNeeded >= 1) { // if we need a purple
            spindexColor[mag_intakeIndex] = intakeColor; // record the color of the ball taken in

          } else {
            intake.eject();
          }

        } else if (intakeColor == BallColor.GREEN) { // if a green is in the intake
          if (greensNeeded >= 1) { // if we need a green
            spindexColor[mag_intakeIndex] = intakeColor; // record the color of the ball taken in

          } else {
            intake.eject();
          }
        }

      } else if (intake.isEjecting()) {
        if (intakeColor == BallColor.EMPTY) {
          if (!intake.intakeTimer.isRunning()) {
            intake.intakeTimer.start();

          } else if (intake.intakeTimer.isFinished()) {
            intake.intake(); // start the intake
          }
        }
      }
      if (!fillMagSorted()) fillingMag = false; // check if there are still empty slots in the mag

    } else if (intake.isIntaking()) { // if the intake is trying to intake a ball
      BallColor intakeColor =
          getIntakeColor(); // get the color of ball (if any) in the intake position
      if (intakeColor != BallColor.EMPTY) { // if a ball is in the intake position
        intake.stop(); // stop the intake
        spindexColor[mag_intakeIndex] = intakeColor; // record the color of the ball taken in
      }

    } else if (intake.isEjecting()) {
      BallColor intakeColor =
          getIntakeColor(); // get the color of ball (if any) in the intake position
      if (intakeColor == BallColor.EMPTY) { // if there isn't a ball in the intake position
        intake.stop(); // stop the intake
      }
    }
  }

  /**
   * @brief starts filling the mag with two purple balls and one green ball
   * @return true if mag had empty slots, false if mag is full or contains more than 2 purples or 1
   *     green
   */
  public boolean fillMagSorted() {
    if (getColorIndex(BallColor.EMPTY) == NULL)
      return false; // if there are no empty slots in the mag, return false
    fillingMag = true;
    purplesNeeded = 2; // if all slots are empty, we need 2 purples
    greensNeeded = 1; // if all slots are empty, we need 1 green
    for (BallColor color : spindexColor) {
      if (color == BallColor.PURPLE)
        purplesNeeded--; // if slot contains purple, decrease number of purples needed by one
      if (color == BallColor.GREEN)
        greensNeeded--; // if slot contains green, decrease number of greens needed by one
    }
    if (purplesNeeded < 0 || greensNeeded < 0)
      return false; // if mag contains more than 2 purples or 1 green, return false
    if (!intake.isBusy()) intake.intake(); // start the intake spinning
    moveSpindexIntake(getColorIndex(BallColor.EMPTY)); // move the spindex to an empty slot
    return true;
  }

  /**
   * @brief intakes a single ball of any color to the first empty intake index in the spindex
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
   * @return true if given index was empty, false if a ball was already in the given index or intake
   *     is busy
   */
  private boolean intakeBallIndex(int index) {
    if ((spindexColor[index] != BallColor.EMPTY) || intake.isBusy())
      return false; // return false if given index contains a ball, or intake is busy

    liftServo.setPosition(liftServoRestPos); // lower lifter out of the way
    moveSpindexIntake(index); // move spindex to correct position
    intake.intake(); // start the intake spinning

    return true;
  }

  /**
   * @brief starts shooting a sequence of balls out of the turret
   * @param sequence the color sequence to shoot
   * @return true if the mag was full, false if the mag isn't full or a sequence is already being
   *     shot
   */
  public boolean shootSequence(BallSequence sequence) {
    if (shootingSequence) return false;
    int purples = 0;
    int greens = 0;
    for (BallColor color : spindexColor) {
      if (color == BallColor.PURPLE) purples++;
      if (color == BallColor.GREEN) greens++;
    }
    if ((purples != 2) || (greens != 1))
      return false; // return false if the wrong number of balls are in the mag
    shootingSequence = true; // a sequence is being shot
    ballSequence = sequence; // store the requested sequence
    sequenceIndex = 0; // start with the first ball in the sequence
    flywheel.activate(); // start flywheel spinning up to full speed
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
  private boolean moveSpindexIntake(int index) {
    double prevPos = spinServo.getPosition(); // get old position
    double newPos = spindexIntake[index]; // get new position
    spinServo.setPosition(newPos); // set servo to new position
    spinServoTimer.start(); // start timer for moving spindex
    mag_intakeIndex = index; // set new intake index
    mag_shootIndex = NULL; // spindex isn't at a shooting index
    return prevPos != newPos; // return true if new position is different from old one
  }

  /**
   * @brief moves the specified spindex index to its shooting position
   * @param index the spindex index to move to the shooting position
   * @return true if the servo moved, false if the servo was already at requested location
   */
  private boolean moveSpindexShoot(int index) {
    double prevPos = spinServo.getPosition(); // get old position
    double newPos = spindexShoot[index]; // get new position
    spinServo.setPosition(newPos); // set servo to new position
    spinServoTimer.start(); // start timer for moving spindex
    mag_shootIndex = index; // set new shooting index
    mag_intakeIndex = NULL; // spindex isn't at an intake index
    return prevPos != newPos; // return true if new position is different from old one
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
