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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

public class ScoringSystem {
  private enum SequenceMode {
    SORTED,
    UNSORTED
  }

  private final ActiveIntake intake; // the intake on the robot
  private final Turret turret; // the flywheel on the robot
  private final Spindex spindex; // the spindex on the robot
  private final Limelight limelight; // the limelight camera on the turret
  private boolean fillingMag = false; // if the mag is being filled
  private SequenceMode fillingMode = SequenceMode.SORTED; // the mode the mag is being filled in
  private boolean emptyingMag = false; // if a sequence is being shot out of the turret
  private SequenceMode emptyingMode = SequenceMode.SORTED; // the mode the mag is being emptied in
  private BallSequence ballSequence; // the sequence being shot
  private final AllianceColor allianceColor; // the alliance we are on
  private final SimpleTimer targetLockTimer;
  private boolean targetVisible = true;
  private boolean turretAimOverride = false; // if the aim of the turret is overridden
  private double horizontalAngleOverride = 0;
  private double distanceOverride = 1;
  private double robotRotationDegrees = 0; // how rotated the robot is, in degrees
  private int sequenceIndex = 0; // the index of ball in the sequence that is being shot
  private int purplesNeeded = 0; // the number of purples needed to fill the mag
  private int greensNeeded = 0; // the number of greens needed to fill the mag
  private boolean clearingIntake =
      false; // if the intake is being reversed to clear a blockage causing a stall
  private final Telemetry telemetry;

  public ScoringSystem(
      ActiveIntake intake,
      Turret turret,
      Spindex spindex,
      Limelight limelight,
      AllianceColor allianceColor,
      Telemetry telemetry) {
    this.intake = intake;
    this.turret = turret;
    this.spindex = spindex;
    this.limelight = limelight;
    this.allianceColor = allianceColor;
    this.targetLockTimer = new SimpleTimer(1); // TODO: tune
    this.telemetry = telemetry;
    this.turret.idle(); // set turret to spin at idle speed
    this.turret.disable(); // don't let the turret spin up
  }

  /**
   * @brief initializes the artifact scoring system
   * @param isPreloaded if the spindex is preloaded with balls
   * @param search if limelight should search for a new ball sequence
   * @note call when OpMode is initialized ("Init" is pressed)
   */
  public void init(boolean isPreloaded, boolean search) {
    spindex.init(BallSequence.GPP, isPreloaded);
    turret.init(0/*search ? allianceColor.getObeliskAngle() : allianceColor.getTargetAngleMin()*/);
    limelight.init(search);
  }

  /**
   * @brief to be called repeatedly while the robot is in init
   */
  public void initLoop() {
    spindex.update();
  }

  /**
   * @brief starts scoring systems up
   * @note call when OpMode is started ("Start" is pressed)
   */
  public void start(boolean search) {
    turret.enable(); // let the flywheel spin up
    limelight.start();
    if (search) limelight.detectSequence();
  }

  /**
   * @brief stops all powered movement as quickly as possible (think E-stop)
   * @note intended to be called when the "Stop" button is pressed
   */
  public void stop() {
    turret.disable(); // stop the flywheel
    turret.update();
    intake.stop(); // stop the intake
  }

  /**
   * @brief updates everything to do with the artifact scoring system
   * @note call each loop
   */
  public void update() {
    limelight.update();
    ballSequence = limelight.getSequence();
    updateAiming();
    updateShooting();
    updateIntake();
    intake.update();
    turret.update();
    spindex.update();
    telemetry.addData("Mag", Arrays.toString(spindex.getSpindexContents()));
    telemetry.addData("Shooting", emptyingMag);
    telemetry.addData("Filling", fillingMag);
    telemetry.addData("Intake current", intake.getAverageCurrentAmps());
    telemetry.addData("Limelight mode", limelight.getMode().toString());
    telemetry.addData("Spindex mode", spindex.getState().toString());
    telemetry.addData("Spinner at target", spindex.spinner.isAtTarget());
  }

  /**
   * @brief updates everything to do with aiming the turret
   */
  private void updateAiming() {
    if (turretAimOverride) {
      turret.setHorizontalAngle(horizontalAngleOverride);
      turret.setTargetDistance(distanceOverride);
      return;

    } else if (!limelight.isReadyToNavigate()) {
      turret.setHorizontalAngle(allianceColor.getObeliskAngle());
      return;
    }

    boolean targetWasVisible = targetVisible;
    targetVisible = limelight.isTargetInFrame();

    if (targetWasVisible && !targetVisible) {
      // ^ if limelight just lost sight of the target
      targetLockTimer.start(); // start timer
    }

    if (limelight.isTargetInFrame()) {
      // set turret distance to target
      turret.setTargetDistance(limelight.getTargetSize());
      // adjust turret angle
      turret.setHorizontalAngle(
          turret.getHorizontalAngleDegrees() + limelight.getTargetOffsetAngleDegrees());

    } else if (targetLockTimer.isFinished() && turret.isAtTarget()) {
      // ^ if turret is moved and limelight hasn't seen the target for long enough
      // re-lock onto apriltag
      double angleMin = allianceColor.getTargetAngleMin() + robotRotationDegrees; // invert?
      double angleMax = allianceColor.getTargetAngleMax() + robotRotationDegrees; // invert?
      double range = angleMax - angleMin;
      double step = range / 6;
      double angleNow = turret.getTargetHorizontalAngleDegrees() + robotRotationDegrees;
      double angleToSet = angleNow + step;

      if (angleToSet > angleMax || angleToSet < angleMin) {
        angleToSet = angleMin;
      }

      turret.setHorizontalAngle(angleToSet);
    }
  }

  /**
   * @brief updates everything to do with shooting balls out of the turret
   */
  private void updateShooting() {
    if (emptyingMag) {
      // ^ if we are emptying the mag (rapid fire)
      boolean emptying = false;
      switch (emptyingMode) {
        case UNSORTED:
          emptying = emptyMagUnsorted();
          break;

        case SORTED:
          emptying = emptyMagSorted();
          break;
      }

      if (!emptying) {
        emptyingMag = false; // we are no longer emptying the mag
        turret.idle(); // let flywheel slow down to idle speed
        spindex.moveSpindexIdle(spindex.getIndex()); // idle the spindex
      }

    } else if (turret.isEnabled() && turret.isActive()) {
      // ^ if we are just shooting one ball
      if (spindex.getState() == Spindex.SpindexState.LIFTED && spindex.isAtTarget()) {
        // ^ if the spindex has lifted the ball
        turret.idle(); // set the flywheel to slow down to idle speeds
        spindex.moveSpindexIdle(spindex.getIndex()); // idle the spindex

      } else if (turret.isReadyToShoot() && spindex.isReadyToShoot()) {
        // ^ if the flywheel is spinning fast enough, and the spindex is ready to shoot
        spindex.liftBall(); // lift ball into turret
      } // if (turret.isUpToSpeed() && spindex.isReadyToShoot())
    } // else if (turret.isEnabled() && turret.isActive())
  } // updateShooting()

  /**
   * @brief updates everything to do with the intake
   */
  private void updateIntake() {
    if (clearingIntake) {
      if (intake.timer.isFinished()) {
        clearingIntake = false;
        intake.intake(); // start the intake up again
      }

    } else if (intake.isStalled() && intake.isIntaking()) {
      // ^ if intake is intaking and stalled
      emergencyEject(); // eject the intake to clear the blockage

    } else if (fillingMag) {
      telemetry.addData("We are filling mag", true);
      // ^ if we are filling the magazine
      if (intake.isIntaking() && spindex.getIsIntakeColorNew() && spindex.isAtTarget()) {
        // ^ if intaking a ball, the spindex is stationary, and a new color of ball is in the intake
        if (spindex.getIntakeColor() == BallColor.PURPLE) {
          // ^ if a purple is in the intake
          if (purplesNeeded >= 1) { // if we need a purple
            spindex.storeIntakeColor(); // record the color of the ball taken in

          } else { // if we don't need a purple
            intake.eject();
            intake.timer.start();
          }

        } else if (spindex.getIntakeColor() == BallColor.GREEN) {
          // ^ if a green is in the intake
          if (greensNeeded >= 1) { // if we need a green
            spindex.storeIntakeColor(); // record the color of the ball taken in

          } else { // if we don't need a green
            intake.eject();
            intake.timer.start();
          }
        }

      } else if (intake.isEjecting() && intake.timer.isFinished()) {
        // ^ if intake is ejecting and the intake timer is done
        intake.intake(); // start the intake
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
      telemetry.addData("We are intaking a ball", true);
      // ^ if the intake is trying to intake a (single) ball
      BallColor intakeColor =
          spindex.getIntakeColor(); // get the color of ball (if any) in the intake position
      if (intakeColor.isShootable()) { // if a ball is in the intake position
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

        default: // if the spindex is in some other mode
          intake.ejectIdle();
          break;
      }

    } else if (spindex.getState() == Spindex.SpindexState.IDLE
        && spindex.isAtTarget()
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
    for (BallColor color : spindex.getSpindexContents()) {
      if (color == BallColor.PURPLE)
        purplesNeeded--; // if slot contains purple, decrease number of purples needed by one
      if (color == BallColor.GREEN)
        greensNeeded--; // if slot contains green, decrease number of greens needed by one
    }
    // if mag contains more than 2 purples or 1 green, return false
    if (purplesNeeded < 0 || greensNeeded < 0) return false;
    return intakeIndex(spindex.getColorIndex(BallColor.EMPTY)); // start intaking balls
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
    return intakeIndex(spindex.getColorIndex(BallColor.EMPTY)); // start intaking balls
  }

  /**
   * @brief intakes a single ball of any color to the first empty intake index in the spindex
   * @return true if a ball was taken in, false if the mag was full or intake is busy
   */
  public boolean intakeBall() {
    // intakeBallIndex() will handle returning false if mag is full
    // intake ball to first empty index
    return intakeIndex(spindex.getColorIndex(BallColor.EMPTY));
  }

  /**
   * @brief intakes a ball at a specified index in the spindex
   * @param index the index in the spindex to put the ball in
   * @return true if given index was empty, false if a ball was already in the given index
   */
  private boolean intakeIndex(int index) {
    // return false if given index contains a ball
    // spindex will return BallColor.INVALID on invalid indexes
    if (spindex.getIndexColor(index).isShootable()) return false;

    spindex.moveSpindexIntake(index); // move spindex to correct position
    intake.intake(); // start the intake spinning

    return true;
  }

  /**
   * @brief shoots all balls in the mag, in sequence order if possible
   * @return true if the mag contained at least one ball, false if the mag is empty
   */
  public boolean shootMag() {
    if (shootSequence()) return true; // try shooting a sequence
    return shootUnsorted(); // try shooting in any order
  }

  /**
   * @brief shoots all balls in the mag in no particular order
   * @return true if the mag contained at least one ball, false if mag is empty
   */
  public boolean shootUnsorted() {
    // if mag is empty, return false
    if (spindex.getColorIndex(BallColor.PURPLE) == NULL
        && spindex.getColorIndex(BallColor.GREEN) == NULL) return false;
    emptyingMag = true; // the mag is being emptied
    emptyingMode = SequenceMode.UNSORTED; // shooting in any order
    int index = NULL;
    for (BallColor color : spindex.getSpindexContents()) { // for each spindex slot
      // note: there should always be a non-empty slot in the spindex if the code gets to this point
      if (color != BallColor.EMPTY) { // if slot isn't empty
        index = spindex.getColorIndex(color); // get index of slot
        break; // don't keep looking for full slots
      }
    }
    shootIndex(index); // shoot a ball from the non-empty slot

    return true; // we are emptying the mag; return true
  }

  /**
   * @brief starts shooting a sequence of balls out of the turret
   * @return true if the mag was full, false if the mag isn't full or a sequence is already being
   *     shot
   */
  public boolean shootSequence() {
    if (emptyingMag) return false;
    int purples = 0;
    int greens = 0;
    for (BallColor color : spindex.getSpindexContents()) {
      if (color == BallColor.PURPLE) purples++;
      if (color == BallColor.GREEN) greens++;
    }
    if ((purples != 2) || (greens != 1))
      return false; // return false if the wrong number of balls are in the mag
    emptyingMag = true; // the mag is being emptied
    emptyingMode = SequenceMode.SORTED; // shooting sorted sequence
    sequenceIndex = 0; // start with the first ball in the sequence
    return emptyMagSorted(); // start emptying mag
  }

  /**
   * @brief sets how rotated the robot is relative to the field
   * @param degrees the angle by which the robot is offset from the field
   */
  public void setRobotRotation(double degrees) {
    robotRotationDegrees = AngleUnit.normalizeDegrees(degrees); // might need to invert
  }

  /**
   * @brief the internal logic for emptying the mag in a sorted manner (shooting a sequence)
   * @return true if the mag is being emptied, false if the mag is empty
   */
  protected boolean emptyMagSorted() {
    int indexToShoot;

    try {
      indexToShoot = spindex.getColorIndex(ballSequence.getBallColors()[sequenceIndex]);
    } catch (IndexOutOfBoundsException e) {
      return false; // done shooting sequence
    }

    if (spindex.isReadyToShoot()) {
      // ^ spindex is ready for ball to be lifted
      if (turret.isReadyToShoot()) {
        // turret is ready for ball to be lifted
        spindex.liftBall();
      }

    } else if (spindex.getState() == Spindex.SpindexState.LIFTED
        || spindex.getIndex() != indexToShoot) {
      // ^ spindex is done lifting ball, or hasn't been set yet
      indexToShoot = spindex.getColorIndex(ballSequence.getBallColors()[sequenceIndex++]);

      if (spindex.isIndexValid(indexToShoot)) {
        // ^ spindex isn't empty
        shootIndex(indexToShoot);

      } else {
        return false; // mag empty
      }
    }

    return true; // emptying mag
  }

  /**
   * @brief the internal logic for emptying the mag in a unsorted manner
   * @return true if the mag is being emptied, false if the mag is empty
   */
  protected boolean emptyMagUnsorted() {
    int indexToShoot = NULL;

    for (BallColor color : spindex.getSpindexContents()) {
      if (color.isShootable()) {
        indexToShoot = spindex.getColorIndex(color);
        break; // don't keep looking
      }
    }

    if (spindex.isReadyToShoot()) {
      // ^ spindex is ready for ball to be lifted
      if (turret.isReadyToShoot()) {
        // turret is ready for ball to be lifted
        spindex.liftBall();
      }

    } else if (spindex.getState() == Spindex.SpindexState.LIFTED
        || (spindex.getIndex() != indexToShoot
            && spindex.getState() != Spindex.SpindexState.SHOOTING)) {
      // ^ spindex is done lifting ball, or hasn't been set yet
      if (spindex.isIndexValid(indexToShoot)) {
        // ^ spindex isn't empty
        shootIndex(indexToShoot);

      } else {
        return false; // mag empty
      }
    }

    return true;
  }

  /**
   * @brief sets the intake to eject at full speed (for some amount of time)
   * @note intended for use in emergency game situations (when something has malfunctioned); not
   *     intended to be used normally or regularly
   */
  public void emergencyEject() {
    intake.eject(); // set intake to eject at full speed
    intake.timer.start(); // start intake timer
    clearingIntake = true; // we are clearing the intake
  }

  /**
   * @brief forces a re-check of the sequence to shoot
   * @note intended for use in emergency game situations (when something has malfunctioned); not
   *     intended to be used normally or regularly
   */
  public void emergencyRecheckSequence() {
    limelight.detectSequence();
  }

  /**
   * @brief sets the turret to 0 degrees
   * @note intended for use at the end of auto
   */
  public void homeTurret() {
    turretAimOverride = true;
    horizontalAngleOverride = 0;
    distanceOverride = 1;
  }

  /**
   * @brief overrides aiming, switching control away from limelight
   * @param distance the distance from the target, as a fraction of limelights view
   * @param angle the angle to turn the turret to
   */
  public void overrideAiming(double distance, double angle) {
    turretAimOverride = true;
    distanceOverride = distance;
    horizontalAngleOverride = angle;
  }

  /**
   * @brief switches aiming control back to limelight (from manual override)
   */
  public void autoAim() {
    turretAimOverride = false;
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
   */
  private void shootIndex(int index) {
    // NOTE: assumes that the servos will move in the time it takes the flywheel to spin up
    turret.activate(); // spin flywheel up to speed
    intake.intakeIdle(); // start intake up to keep balls in the mag
    spindex.moveSpindexShoot(index); // move spindex to correct position
  }
}
