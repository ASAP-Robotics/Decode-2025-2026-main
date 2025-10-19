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

import static org.firstinspires.ftc.teamcode.types.Helpers.NULL;

import java.util.Arrays;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;

public class ScoringSystem {
  private final ActiveIntake intake; // /< the intake on the robot
  private final Turret turret; // /< the flywheel on the robot
  private final Spindex spindex; // /< the spindex on the robot
  private final Camera camera; // /< the camera on the turret
  private boolean fillingMag = false; // /< if the mag is being filled
  private boolean shootingSequence = false; // /< if a sequence is being shot out of the turret
  private BallSequence ballSequence; // /< the sequence being shot
  private int sequenceIndex = 0; // /< the index of ball in the sequence that is being shot
  private int purplesNeeded = 0; // the number of purples needed to fill the mag
  private int greensNeeded = 0; // the number of greens needed to fill the mag
  private final Telemetry telemetry;

  public ScoringSystem(
      ActiveIntake intake, Turret turret, Spindex spindex, Camera camera, Telemetry telemetry) {
    this.intake = intake;
    this.turret = turret;
    this.spindex = spindex;
    this.camera = camera;
    this.telemetry = telemetry;
    this.turret.idle(); // set turret to spin at idle speed
    this.turret.disable(); // don't let the turret spin up
    this.init(); // initialize scoring systems
  }

  /**
   * @brief initializes the artifact scoring system
   * @note call when OpMode is initialized ("Init" is pressed)
   */
  public void init() {
    spindex.init();
  }

  /**
   * @brief starts scoring systems up
   * @note call when OpMode is started ("Start" is pressed)
   */
  public void start() {
    turret.enable(); // let the flywheel spin up
  }

  /**
   * @brief stops all movement as quickly as possible (think E-stop)
   * @note intended to be called when the "Stop" button is pressed
   */
  public void stop() {
    turret.disable(); // stop the flywheel
    intake.stop(); // stop the intake
  }

  /**
   * @brief updates everything to do with the artifact scoring system
   * @note call each loop
   */
  public void update() {
    updateAiming();
    updateShooting();
    updateIntake();
    turret.update();
    spindex.update();
    telemetry.addData("Mag", Arrays.toString(spindex.getSpindexColor()));
    telemetry.addData("Shooting", shootingSequence);
    telemetry.addData("Filling", fillingMag);
  }

  /**
   * @brief updates everything to do with aiming the turret
   */
  private void updateAiming() {
    turret.setTargetDistance(camera.getNavigationAprilTagDistance());
    turret.setHorizontalAngle(turret.getTargetHorizontalAngleDegrees() +
        camera.getNavigationAprilTagAngleX());
    turret.setVerticalAngle(turret.getTargetVerticalAngleDegrees() +
        camera.getNavigationAprilTagAngleY());
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
      telemetry.addData("Contains ball", turret.getContainsBall());
      if (turret.getContainsBall()) { // if there is a ball in the flywheel
        if (turret.shotTimer.isFinished()) { // if the ball entered the flywheel long enough ago
          turret.setContainsBall(false); // there is not a ball in the flywheel
          spindex.setShootingIndexEmpty(); // spindex slot is now empty
          if (sequenceIndex < (sequence.length - 1)) { // if the sequence isn't done
            sequenceIndex++; // move on to the next ball in the sequence

          } else { // if the sequence is done
            shootingSequence = false; // we are no longer shooting a sequence
            turret.idle(); // let flywheel slow down to idle speed
          }
        }

      } else { // if there isn't a ball in the flywheel
        int shootingColorIndex = spindex.getColorIndex(shootingColor);
        if (spindex.getIsSpindexMoved()
            && (spindex.getShootIndex()
                != shootingColorIndex)) { // if the spindex position hasn't been set
          spindex.moveSpindexShoot(shootingColorIndex); // move spindex to correct location

        } else if (turret.isUpToSpeed()
            && spindex
                .getIsSpindexMoved()) { // if the flywheel is up to speed and the spindex is done
          // moving
          spindex.liftBall(); // lift ball into flywheel
          turret.setContainsBall(true); // flywheel now has a ball in it
          turret.shotTimer.start();
        }
      }

    } else if (turret.isEnabled() && turret.isActive()) { // if we are just shooting one ball
      if (turret.getContainsBall()) {
        if (turret.shotTimer.isFinished()) {
          turret.setContainsBall(false); // the ball should be out of the flywheel
          turret.idle(); // set the flywheel to slow down to idle speeds
        }

      } else {
        if (turret.isUpToSpeed()) {
          spindex.liftBall(); // lift ball into flywheel
          turret.setContainsBall(true);
        }
      }
    }
  }

  /**
   * @brief updates everything to do with the intake
   */
  private void updateIntake() {
    if (fillingMag) { // if we are filling the magazine
      if (intake.isIntaking() && spindex.getIsIntakeColorNew()) {
        if (spindex.getIntakeColor() == BallColor.PURPLE) { // if a purple is in the intake
          if (purplesNeeded >= 1) { // if we need a purple
            spindex.storeIntakeColor(); // record the color of the ball taken in

          } else {
            intake.eject();
          }

        } else if (spindex.getIntakeColor() == BallColor.GREEN) { // if a green is in the intake
          if (greensNeeded >= 1) { // if we need a green
            spindex.storeIntakeColor(); // record the color of the ball taken in

          } else {
            intake.eject();
          }
        }

      } else if (intake.isEjecting()) {
        if (spindex.getIntakeColor() == BallColor.EMPTY) {
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
          spindex.getIntakeColor(); // get the color of ball (if any) in the intake position
      if (intakeColor != BallColor.EMPTY) { // if a ball is in the intake position
        intake.stop(); // stop the intake
        spindex.storeIntakeColor(); // record the color of the ball taken in
      }

    } else if (intake.isEjecting()) {
      BallColor intakeColor =
          spindex.getIntakeColor(); // get the color of ball (if any) in the intake position
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
    if (spindex.getColorIndex(BallColor.EMPTY) == NULL)
      return false; // if there are no empty slots in the mag, return false
    fillingMag = true;
    purplesNeeded = 2; // if all slots are empty, we need 2 purples
    greensNeeded = 1; // if all slots are empty, we need 1 green
    for (BallColor color : spindex.getSpindexColor()) {
      if (color == BallColor.PURPLE)
        purplesNeeded--; // if slot contains purple, decrease number of purples needed by one
      if (color == BallColor.GREEN)
        greensNeeded--; // if slot contains green, decrease number of greens needed by one
    }
    if (purplesNeeded < 0 || greensNeeded < 0)
      return false; // if mag contains more than 2 purples or 1 green, return false
    if (!intake.isBusy()) intake.intake(); // start the intake spinning
    spindex.moveSpindexIntake(
        spindex.getColorIndex(BallColor.EMPTY)); // move the spindex to an empty slot
    return true;
  }

  /**
   * @brief intakes a single ball of any color to the first empty intake index in the spindex
   * @return true if a ball was taken in, false if the mag was full or intake is busy
   */
  public boolean intakeBall() {
    int index = NULL;
    BallColor[] spindexColor = spindex.getSpindexColor();
    for (int i = 0; i < spindexColor.length; i++) {
      if (spindexColor[i] == BallColor.EMPTY) index = i;
    }

    if (index == NULL) return false; // if there are no empty slots, return false

    return intakeBallIndex(
        spindex.getIntakeIndex()); // intake ball to index currently at the intake
  }

  /**
   * @brief intakes a ball at a specified index in the spindex
   * @param index the index in the spindex to put the ball in
   * @return true if given index was empty, false if a ball was already in the given index or intake
   *     is busy
   */
  private boolean intakeBallIndex(int index) {
    BallColor[] spindexColor = spindex.getSpindexColor();
    if ((spindexColor[index] != BallColor.EMPTY) || intake.isBusy())
      return false; // return false if given index contains a ball, or intake is busy

    spindex.moveSpindexIntake(index); // move spindex to correct position
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
    for (BallColor color : spindex.getSpindexColor()) {
      if (color == BallColor.PURPLE) purples++;
      if (color == BallColor.GREEN) greens++;
    }
    if ((purples != 2) || (greens != 1))
      return false; // return false if the wrong number of balls are in the mag
    shootingSequence = true; // a sequence is being shot
    ballSequence = sequence; // store the requested sequence
    sequenceIndex = 0; // start with the first ball in the sequence
    turret.activate(); // start flywheel spinning up to full speed
    return true;
  }

  public double setTargetDistance(double inches) {
    return turret.setTargetDistance(inches);
  }

  /**
   * @brief shoots a requested color, if possible
   * @param color the color to be shot
   * @return true if ball shot, false if no balls of requested color are in mag
   */
  public boolean shootColor(BallColor color) {
    int index = spindex.getColorIndex(color); // find index in the spindex of requested color
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
    turret.activate(); // spin flywheel up to speed
    spindex.moveSpindexShoot(index); // move spindex to correct position
  }
}
