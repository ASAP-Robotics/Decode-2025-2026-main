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
  private enum SequenceMode {
    SORTED,
    UNSORTED
  }

  private final ActiveIntake intake; // the intake on the robot
  private final Turret turret; // the flywheel on the robot
  private final Spindex spindex; // the spindex on the robot
  private final Camera camera; // the camera on the turret
  private boolean fillingMag = false; // if the mag is being filled
  private SequenceMode fillingMode = SequenceMode.SORTED; // the mode the mag is being filled in
  private boolean emptyingMag = false; // if a sequence is being shot out of the turret
  private SequenceMode emptyingMode = SequenceMode.SORTED; // the mode the mag is being emptied in
  private BallSequence ballSequence; // the sequence being shot
  private int sequenceIndex = 0; // the index of ball in the sequence that is being shot
  private int purplesNeeded = 0; // the number of purples needed to fill the mag
  private int greensNeeded = 0; // the number of greens needed to fill the mag
  private final Telemetry telemetry;

  public ScoringSystem(
      ActiveIntake intake,
      Turret turret,
      Spindex spindex,
      Camera camera,
      BallSequence sequence,
      Telemetry telemetry) {
    this.intake = intake;
    this.turret = turret;
    this.spindex = spindex;
    this.camera = camera;
    this.ballSequence = sequence;
    this.telemetry = telemetry;
    this.turret.idle(); // set turret to spin at idle speed
    this.turret.disable(); // don't let the turret spin up
  }

  /**
   * @brief initializes the artifact scoring system
   * @param isPreloaded if the spindex is preloaded with balls
   * @note call when OpMode is initialized ("Init" is pressed)
   */
  public void init(boolean isPreloaded) {
    spindex.init(isPreloaded ? BallSequence.GPP : null, isPreloaded);
  }

  /**
   * @brief starts scoring systems up
   * @note call when OpMode is started ("Start" is pressed)
   */
  public void start() {
    turret.enable(); // let the flywheel spin up
  }

  /**
   * @brief stops all powered movement as quickly as possible (think E-stop)
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
    telemetry.addData("Shooting", emptyingMag);
    telemetry.addData("Filling", fillingMag);
  }

  /**
   * @brief updates everything to do with aiming the turret
   */
  private void updateAiming() {
    turret.setTargetDistance(camera.getNavigationAprilTagDistance());
    turret.setHorizontalAngle(
        turret.getTargetHorizontalAngleDegrees() + camera.getNavigationAprilTagAngleX());
  }

  /**
   * @brief updates everything to do with shooting balls out of the turret
   */
  private void updateShooting() {
    if (emptyingMag) { // if we are emptying the mag (rapid fire)
      boolean isSorted = (emptyingMode == SequenceMode.SORTED); // if we are shooting a sequence

      if (turret.getContainsBall()) { // if there is a ball in the turret
        if (turret.shotTimer.isFinished()) { // if the ball is now out of the turret
          turret.setContainsBall(false); // there is not a ball in the turret

          if (isSorted) { // if shooting a sequence
            BallColor[] sequence = ballSequence.getBallColors();
            if (sequenceIndex < (sequence.length - 1)) { // if the sequence isn't done
              sequenceIndex++; // move on to the next ball in the sequence

            } else { // if the sequence is done
              emptyingMag = false; // we are no longer shooting a sequence
              turret.idle(); // let flywheel slow down to idle speed
              spindex.moveSpindexIdle(spindex.getIndex()); // idle the spindex
            }

          } else { // if emptying the mag in no particular order
            int numBallsLeft = 0;
            for (BallColor color : spindex.getSpindexColor()) {
              if (color != BallColor.EMPTY) numBallsLeft++;
            }

            if (numBallsLeft == 0) { // if the mag is empty
              emptyingMag = false; // we are no longer emptying the mag
              turret.idle(); // let flywheel slow down to idle speed
              spindex.moveSpindexIdle(spindex.getIndex()); // idle the spindex
            } // if (numBallsLeft == 0)
          } // else ( if (isSorted) )
        } // if (turret.shotTimer.isFinished())

      } else { // if there isn't a ball in the turret
        int shootingColorIndex = NULL;

        if (isSorted) { // if shooting a sequence
          BallColor[] sequence = ballSequence.getBallColors();
          BallColor shootingColor = sequence[sequenceIndex];
          shootingColorIndex = spindex.getColorIndex(shootingColor);

        } else { // if emptying the mag in no particular order
          for (BallColor color : spindex.getSpindexColor()) { // for each spindex slot
            if (color != BallColor.EMPTY) { // if it isn't empty
              shootingColorIndex = spindex.getColorIndex(color); // shoot this ball
              break; // we don't need to keep looking
            }
          }

          if (shootingColorIndex == NULL) { // if the mag is empty
            // this should never happen, but just in case
            emptyingMag = false; // we are no longer emptying the mag
            turret.idle(); // let flywheel slow down to idle speed
            spindex.moveSpindexIdle(spindex.getIndex()); // idle the spindex
            return; // we don't need to finish the rest of the logic
          }
        }

        if (spindex.getIndex() != shootingColorIndex) { // if the spindex position hasn't been set
          spindex.moveSpindexShoot(shootingColorIndex); // move spindex to correct location

        } else if (turret.isUpToSpeed() && spindex.isReadyToShoot()) {
          // ^ if the flywheel is up to speed and the spindex is ready to shoot
          spindex.liftBall(); // lift ball into turret
          turret.setContainsBall(true); // turret now has a ball in it
          turret.shotTimer.start(); // start the shot timer
        }
      }

    } else if (turret.isEnabled() && turret.isActive()) { // if we are just shooting one ball
      if (turret.getContainsBall()) { // if the turret contains a ball
        if (turret.shotTimer.isFinished()) { // if the turret no longer contains a ball
          turret.setContainsBall(false); // the ball should be out of the flywheel
          turret.idle(); // set the flywheel to slow down to idle speeds
          spindex.moveSpindexIdle(spindex.getIndex()); // idle the spindex
        }

      } else { // if the turret doesn't contain a ball yet
        // if the flywheel is spinning fast enough, and the spindex is ready to shoot
        if (turret.isUpToSpeed() && spindex.isReadyToShoot()) {
          spindex.liftBall(); // lift ball into turret
          turret.setContainsBall(true); // turret now contains a ball
          turret.shotTimer.start(); // start timer for shot duration
        } // if (turret.isUpToSpeed())
      } // else ( if (turret.getContainsBall()) )
    } // else if (turret.isEnabled() && turret.isActive())
  } // updateShooting()

  /**
   * @brief updates everything to do with the intake
   */
  private void updateIntake() {
    // if we are filling the magazine
    if (fillingMag) {
      // if intaking a ball, the spindex is stationary, and a new color of ball is in the intake
      if (intake.isIntaking() && spindex.getIsIntakeColorNew() && spindex.getIsSpindexMoved()) {
        // if a purple is in the intake
        if (spindex.getIntakeColor() == BallColor.PURPLE) {
          // if we need a purple
          if (purplesNeeded >= 1) {
            spindex.storeIntakeColor(); // record the color of the ball taken in

          } else {
            intake.eject();
            intake.timer.start();
          }

        } else if (spindex.getIntakeColor() == BallColor.GREEN) { // if a green is in the intake
          if (greensNeeded >= 1) { // if we need a green
            spindex.storeIntakeColor(); // record the color of the ball taken in

          } else {
            intake.eject();
            intake.timer.start();
          }
        }

      } else if (intake.isEjecting()) {
        if (spindex.getIntakeColor() == BallColor.EMPTY) {
          if (intake.timer.isFinished()) {
            intake.intake(); // start the intake
          }
        }
      }

      switch (fillingMode) {
        case SORTED:
          // check if there are still empty slots in the mag (and move spindex to an empty slot)
          if (!fillMagSorted()) {
            fillingMag = false;
            intake.intakeIdle(); // start intake up to keep balls in the mag
            spindex.moveSpindexIdle(spindex.getIndex()); // move spindex to idle position
          }
          break;

        case UNSORTED:
          // check if there are still empty slots in the mag (and move spindex to an empty slot)
          if (!fillMagUnsorted()) {
            fillingMag = false;
            intake.intakeIdle(); // start intake up to keep balls in the mag
            spindex.moveSpindexIdle(spindex.getIndex()); // move spindex to idle position
          }
          break;
      }

    } else if (intake.isIntaking() && !intake.isIdling()) {
      // ^ if the intake is trying to intake a (single) ball
      BallColor intakeColor =
          spindex.getIntakeColor(); // get the color of ball (if any) in the intake position
      if (intakeColor != BallColor.EMPTY) { // if a ball is in the intake position
        intake.stop(); // stop the intake
        spindex.storeIntakeColor(); // record the color of the ball taken in
      }

    } else if (intake.isEjecting() && !intake.isIdling()) {
      // ^ if the intake is trying to eject a (single) ball
      switch (spindex.getState()) {
        case INTAKING: // if the spindex is in intaking mode
          // if there isn't a ball in the intake position
          if (spindex.getIntakeColor() == BallColor.EMPTY) {
            intake.ejectIdle(); // slow intake down; the ball is out
          }
          break;

        case IDLE: // if the spindex is in idle mode
          // if the ejecting timer is done
          if (intake.timer.isFinished()) {
            intake.ejectIdle(); // slow intake down; the ball is out
          }
          break;

        default:
          intake.ejectIdle();
          break;
      }

    } else if (spindex.getState() == Spindex.SpindexState.IDLE
        && spindex.getIsSpindexMoved()
        && intake.isIntaking()) {
      // ^ if the spindex is stationary at an idle position and the intake is intaking
      intake.eject(); // eject any balls in the intake
      intake.timer.start(); // start the timer for ejecting the ball
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
    fillingMode = SequenceMode.SORTED; // fill the mag with sorted balls
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
    if (!intake.isBusy() || intake.isIdling()) intake.intake(); // start the intake spinning
    spindex.moveSpindexIntake(
        spindex.getColorIndex(BallColor.EMPTY)); // move the spindex to an empty slot
    return true;
  }

  /**
   * @brief starts filling the mag with three balls of any color
   * @return true if the mag had empty slots, false if the mag is full
   */
  public boolean fillMagUnsorted() {
    if (spindex.getColorIndex(BallColor.EMPTY) == NULL)
      return false; // if there are no empty slots in the mag, return false
    fillingMag = true;
    fillingMode = SequenceMode.UNSORTED; // fill the mag with unsorted balls
    purplesNeeded = 3; // we could intake up to 3 greens
    greensNeeded = 3; // we could intake up to 3 purples
    intake.intake(); // start the intake spinning
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
        spindex.getColorIndex(BallColor.EMPTY)); // intake ball to first empty index
  }

  /**
   * @brief intakes a ball at a specified index in the spindex
   * @param index the index in the spindex to put the ball in
   * @return true if given index was empty, false if a ball was already in the given index or intake
   *     is busy
   */
  private boolean intakeBallIndex(int index) {
    if ((spindex.getIndexColor(index) != BallColor.EMPTY) || intake.isBusy())
      return false; // return false if given index contains a ball, or intake is busy

    spindex.moveSpindexIntake(index); // move spindex to correct position
    intake.intake(); // start the intake spinning

    return true;
  }

  /**
   * @brief shoots all balls in the mag, in sequence order if possible
   * @return true if the mag contained at least one ball, false if the mag is empty
   * @note if the balls in the mag are not sorted, they will be shot unsorted. If the mag contains
   *     exactly one green and two purple balls, the last shot sequence will be shot
   */
  public boolean shootMag() {
    return shootMag(ballSequence); // default to last shot sequence
  }

  /**
   * @brief shoots all balls in the mag, in sequence order if possible
   * @return true if the mag contained at least one ball, false if the mag is empty
   * @note if the balls in the mag are not sorted, they will be shot unsorted. If the mag contains
   *     exactly one green and two purple balls, the specified sequence will be shot
   */
  public boolean shootMag(BallSequence sequence) {
    if (shootSequence(ballSequence)) return true; // try shooting the last shot sequence
    int fullSlots = 0;
    for (BallColor color : spindex.getSpindexColor()) { // for each spindex slot
      if (color != BallColor.EMPTY) fullSlots++; // if it isn't empty
    }
    if (fullSlots == 0) return false; // if mag is empty, return false
    emptyingMag = true; // the mag is being emptied
    emptyingMode = SequenceMode.UNSORTED; // shooting in any order
    ballSequence = sequence;
    turret.activate(); // start the flywheel spinning up to full speed
    intake.intakeIdle(); // start intake up to keep balls in the mag
    return true; // we are emptying the mag; return true
  }

  /**
   * @brief starts shooting the lsat shot sequence of balls out of the turret
   * @return true if the mag was full, false if the mag isn't full or a sequence is already being
   *     shot
   */
  public boolean shootSequence() {
    return shootSequence(ballSequence); // use last shot sequence
  }

  /**
   * @brief starts shooting a sequence of balls out of the turret
   * @param sequence the color sequence to shoot
   * @return true if the mag was full, false if the mag isn't full or a sequence is already being
   *     shot
   */
  public boolean shootSequence(BallSequence sequence) {
    if (emptyingMag) return false;
    int purples = 0;
    int greens = 0;
    for (BallColor color : spindex.getSpindexColor()) {
      if (color == BallColor.PURPLE) purples++;
      if (color == BallColor.GREEN) greens++;
    }
    if ((purples != 2) || (greens != 1))
      return false; // return false if the wrong number of balls are in the mag
    emptyingMag = true; // the mag is being emptied
    emptyingMode = SequenceMode.SORTED; // shooting sorted sequence
    ballSequence = sequence; // store the requested sequence
    sequenceIndex = 0; // start with the first ball in the sequence
    turret.activate(); // start flywheel spinning up to full speed
    intake.intakeIdle(); // start intake up to keep balls in the mag
    return true;
  }

  /**
   * @brief sets the intake to eject at full speed (for some amount of time)
   * @note intended for use in emergency game situations (when something has malfunctioned); not
   *     intended to be used normally or regularly
   */
  public void emergencyEject() {
    intake.eject();
    intake.timer.start();
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
    intake.intakeIdle(); // start intake up to keep balls in the mag
    spindex.moveSpindexShoot(index); // move spindex to correct position
  }
}
