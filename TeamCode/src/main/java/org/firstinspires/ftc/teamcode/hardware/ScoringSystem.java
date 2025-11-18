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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;

public class ScoringSystem {
  public enum State {
    UNINITIALISED,
    WAITING_FOR_START,
    EMPTY,
    FULL,
    FILLING,
    EMPTYING
  }

  protected enum SequenceMode {
    SORTED,
    UNSORTED
  }

  private final ActiveIntake intake; // the intake on the robot
  private final Turret turret; // the flywheel on the robot
  private final Spindex spindex; // the spindex on the robot
  private final Limelight limelight; // the limelight camera on the turret
  protected State state = State.UNINITIALISED; // the state of the scoring system
  protected SequenceMode sequenceMode = SequenceMode.UNSORTED; // the mode used for shooting
  private BallSequence ballSequence = BallSequence.PPG; // the sequence being shot
  private final AllianceColor allianceColor; // the alliance we are on
  private boolean turretAimOverride = false; // if the aim of the turret is overridden
  private double horizontalAngleOverride = 0;
  private boolean tuning = false;
  private double verticalAngleOverride = 60;
  private double rpmOverride = 2000;
  private double distanceOverride = 1;
  private final Pose2D targetPosition; // the position of the target to shoot at
  private Pose2D robotPosition =
      new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0); // the position of the robot
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
    this.telemetry = telemetry;
    this.targetPosition = this.allianceColor.getTargetLocation();
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
    turret.init(search ? allianceColor.getObeliskAngle() : getRelativeTargetAngle());
    limelight.init(search);
    state = State.WAITING_FOR_START;
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
  public void start(boolean isPreloaded, boolean search) {
    turret.enable(); // let the flywheel spin up
    limelight.start();
    if (search) limelight.detectSequence();
    state = isPreloaded ? State.FULL : State.EMPTY;
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
    telemetry.addData("State", state.toString());
    telemetry.addData("Locked", limelight.isTargetInFrame() && turret.isAtTarget());
    telemetry.addData("Sequence", ballSequence.toString());
  }

  /**
   * @brief updates everything to do with aiming the turret
   */
  private void updateAiming() {
    if (tuning) {
      turret.tuneRpm(rpmOverride);
      turret.setVerticalAngle(verticalAngleOverride);

    } else if (turretAimOverride) {
      turret.setHorizontalAngle(horizontalAngleOverride);
      turret.setTargetDistance(distanceOverride);
      return;

    } else if (!limelight.isReadyToNavigate()) {
      turret.setHorizontalAngle(allianceColor.getObeliskAngle());
      return;
    }

    turret.setHorizontalAngle(getRelativeTargetAngle());
    turret.setTargetDistance(getTargetDistance());
  }

  /**
   * @brief updates everything to do with shooting balls out of the turret
   */
  private void updateShooting() {
    if (state != State.EMPTYING) return;
    // ^ only do this logic if emptying

    boolean emptying = false;
    switch (sequenceMode) {
      case UNSORTED:
        emptying = emptyMagUnsorted();
        break;

      case SORTED:
        emptying = emptyMagSorted();
        break;
    }

    if (!emptying) {
      state = State.EMPTY; // we are no longer emptying the mag
      turret.idle(); // let flywheel slow down to idle speed
      spindex.moveSpindexIdle(spindex.getIndex()); // idle the spindex
    }
  } // updateShooting()

  /**
   * @brief updates everything to do with the intake
   */
  private void updateIntake() {
    if (clearingIntake) {
      // ^ if clearing the intake
      if (intake.timer.isFinished()) {
        // ^ if done clearing the intake
        clearingIntake = false;
        switch (state) {
          case UNINITIALISED:
          case WAITING_FOR_START:
            return;

          case FULL:
            intake.ejectIdle();
            return;

          case EMPTYING:
            intake.intakeIdle();
            return;

          default:
            intake.intake();
            break;
        }

      } else {
        return;
      }

    } else if (intake.isStalled() && intake.isIntaking()) {
      // ^ if intake is intaking and stalled
      clearIntake(); // eject the intake to clear the blockage
      return;
    }

    switch (state) {
      case UNINITIALISED:
      case WAITING_FOR_START:
        break;

      case FULL:
        if (spindex.isAtTarget() && intake.isIntaking())
          clearIntake();
        break;

      case EMPTY:
        fillMag();
        break;

      case FILLING:
        if (spindex.getIsIntakeColorNew() && spindex.isAtTarget()) {
          // ^ if intaking a ball, the spindex is stationary, and a new color of ball is in the intake
          if (spindex.getIntakeColor() == BallColor.PURPLE) {
            // ^ if a purple is in the intake
            if (purplesNeeded >= 1) { // if we need a purple
              spindex.storeIntakeColor(); // record the color of the ball taken in

            } else { // if we don't need a purple
              clearIntake();
            }

          } else if (spindex.getIntakeColor() == BallColor.GREEN) {
            // ^ if a green is in the intake
            if (greensNeeded >= 1) { // if we need a green
              spindex.storeIntakeColor(); // record the color of the ball taken in

            } else { // if we don't need a green
              clearIntake();
            }
          }
        }

        if (!fillMag()) {
          state = State.FULL;
          intake.intakeIdle(); // start intake up to keep balls in the mag
          spindex.moveSpindexIdle(spindex.getIndex()); // move spindex to idle position
        }

        break;
    }
  }

  /**
   * @brief starts filling the mag with two purple balls and one green ball
   * @return true if mag had empty slots, false if mag is full or contains more than 2 purples or 1
   *     green
   */
  @Deprecated
  public boolean fillMagSorted() {
    if (spindex.getColorIndex(BallColor.EMPTY) == NULL ||
        (state != State.EMPTY && state != State.FILLING)) return false;
    // ^ if mag isn't empty or being filled, or there are no empty slots in the mag, return false
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
    state = State.FILLING;
    sequenceMode = SequenceMode.SORTED; // fill the mag with sorted balls
    return intakeIndex(spindex.getColorIndex(BallColor.EMPTY)); // start intaking balls
  }

  /**
   * @brief starts filling the mag with three balls of any color
   * @return true if the mag had empty slots, false if the mag is full
   */
  public boolean fillMag() {
    if (spindex.getColorIndex(BallColor.EMPTY) == NULL ||
        (state != State.EMPTY && state != State.FILLING)) return false;
    // ^ if mag isn't empty or being filled, or there are no empty slots in the mag, return false
    state = State.EMPTYING;
    sequenceMode = SequenceMode.UNSORTED; // fill the mag with unsorted balls
    purplesNeeded = 3; // we could intake up to 3 greens
    greensNeeded = 3; // we could intake up to 3 purples
    return intakeIndex(spindex.getColorIndex(BallColor.EMPTY)); // start intaking balls
  }

  /**
   * @brief intakes a single ball of any color to the first empty intake index in the spindex
   * @return true if a ball was taken in, false if the mag was full or intake is busy
   */
  @Deprecated
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
    // return false if given index contains a ball or shooting / full
    // spindex will return BallColor.INVALID on invalid indexes
    if (spindex.getIndexColor(index).isShootable() ||
        (state != State.EMPTY && state != State.FILLING)) return false;

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
    if ((spindex.getColorIndex(BallColor.PURPLE) == NULL
        && spindex.getColorIndex(BallColor.GREEN) == NULL) || state != State.FULL) return false;
    state = State.EMPTYING; // the mag is being emptied
    sequenceMode = SequenceMode.UNSORTED; // shooting in any order
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
    if (state != State.FULL) return false;
    int purples = 0;
    int greens = 0;
    for (BallColor color : spindex.getSpindexContents()) {
      if (color == BallColor.PURPLE) purples++;
      if (color == BallColor.GREEN) greens++;
    }
    if ((purples != 2) || (greens != 1))
      return false; // return false if the wrong number of balls are in the mag
    state = State.EMPTYING; // the mag is being emptied
    sequenceMode = SequenceMode.SORTED; // shooting sorted sequence
    sequenceIndex = 0; // start with the first ball in the sequence
    return emptyMagSorted(); // start emptying mag
  }

  /**
   * @brief gets if the scoring system is idle
   * @return true if the scoring system is idle, false if intaking or shooting
   */
  public boolean isIdle() {
    return state == State.EMPTY;
  }

  /**
   * @brief gets the state of the scoring system
   * @return the state of the scoring system
   */
  public State getState() {
    return state;
  }

  /**
   * @brief updates the current position of the robot on the field
   * @param position the current position of the robot on the field
   * @note intended to be called every loop
   */
  public void setRobotPosition(Pose2D position) {
    robotPosition = position;
  }

  public void setBallSequence(BallSequence sequence) {
    ballSequence = sequence;
  }

  /**
   * @brief the internal logic for emptying the mag in a sorted manner (shooting a sequence)
   * @return true if the mag is being emptied, false if the mag is empty
   */
  protected boolean emptyMagSorted() {
    int indexToShoot;

    try {
      indexToShoot = spindex.getColorIndex(ballSequence.getBallColors()[sequenceIndex]);
    } catch (Exception e) {
      return false; // done shooting sequence
    }

    shootIndex(indexToShoot);

    if (spindex.isReadyToShoot()) {
      // ^ spindex is ready for ball to be lifted
      if (turret.isReadyToShoot()) {
        // turret is ready for ball to be lifted
        spindex.liftBall();
      }

    } else if (spindex.getState() == Spindex.SpindexState.LIFTED) {
      // ^ spindex is done lifting ball, or hasn't been set yet
      try {
        indexToShoot = spindex.getColorIndex(ballSequence.getBallColors()[++sequenceIndex]);
      } catch (Exception e) {
        return false; // done shooting sequence
      }

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
   * @brief gets the distance to the target (from the robot), in inches
   * @return the number of inches from the robot to the target
   */
  protected double getTargetDistance() {
    double distX = targetPosition.getX(DistanceUnit.INCH) - robotPosition.getX(DistanceUnit.INCH);
    double distY = targetPosition.getY(DistanceUnit.INCH) - robotPosition.getY(DistanceUnit.INCH);
    return Math.hypot(Math.abs(distX), Math.abs(distY));
  }

  /**
   * @brief gets the absolute angle from the robot to the target, not accounting for the robots rotation
   * @return the absolute angle from the robot to the target, in degrees
   */
  protected double getAbsoluteTargetAngle() {
    double distX = robotPosition.getX(DistanceUnit.INCH) - targetPosition.getX(DistanceUnit.INCH);
    double distY = robotPosition.getY(DistanceUnit.INCH) - targetPosition.getY(DistanceUnit.INCH);

    return AngleUnit.DEGREES.fromRadians(Math.asin(distY / distX)); // might need to invert things
  }

  /**
   * @brief gets the angle to the target relative to the robot, in degrees
   * @return the angle of the target relative to the robot
   */
  protected double getRelativeTargetAngle() {
    return getAbsoluteTargetAngle() - robotPosition.getHeading(AngleUnit.DEGREES);
  }

  /**
   * @brief sets the intake to eject at full speed (for some amount of time)
   * @note intended for external use only in emergency game situations (when something has
   *     malfunctioned); not intended for external use normally or regularly
   */
  public void clearIntake() {
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
   * @brief gets the 2D position of the robot on the field according to limelight
   * @return the position of the robot, or null if target isn't visible
   */
  public Pose2D getRobotPosition() {
    Pose2D limelightPosition = limelight.getPosition();
    if (limelightPosition == null) return null;
    double x = limelightPosition.getX(DistanceUnit.INCH);
    double y = limelightPosition.getY(DistanceUnit.INCH);
    double heading =
        limelightPosition.getHeading(AngleUnit.DEGREES) + turret.getHorizontalAngleDegrees();
    return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
  }

  /**
   * @brief sets the turret to 0 degrees
   * @note intended for use at the end of auto
   */
  @Deprecated
  public void homeTurret() {
    turretAimOverride = true;
    horizontalAngleOverride = 0;
    distanceOverride = 0;
  }

  /**
   * @brief overrides aiming, switching control away from pinpoint / limelight
   * @param distance the distance from the target, in inches
   * @param angle the angle to turn the turret to
   */
  public void overrideAiming(double distance, double angle) {
    turretAimOverride = true;
    distanceOverride = distance;
    horizontalAngleOverride = angle;
  }

  /**
   * @brief intended for use when tuning the lookup table, providing manual flywheel control
   * @param RPM the RPM to spin the flywheel at
   * @param angle the angle to move the flap to
   */
  public void tuneAiming(double RPM, double angle) {
    tuning = true;
    rpmOverride = RPM;
    verticalAngleOverride = angle;
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
