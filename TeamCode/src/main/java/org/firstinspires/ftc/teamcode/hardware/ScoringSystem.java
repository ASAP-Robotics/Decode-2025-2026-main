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
import org.jetbrains.annotations.TestOnly;

public class ScoringSystem {
  public enum State {
    UNINITIALISED,
    FULL,
    INTAKING,
    SHOOTING
  }

  protected enum SequenceMode {
    SORTED,
    HALF_SORT,
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
  private boolean clearingIntake = false; // if the intake is being reversed to clear a blockage
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
    turret.setActive(!isPreloaded);
    turret.enable();
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
  public void start(boolean isPreloaded, boolean search) {
    turret.enable(); // let the flywheel spin up
    limelight.start();
    if (search) limelight.detectSequence();
    state = isPreloaded ? State.FULL : State.INTAKING;
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
    telemetry.addData("Position", robotPosition);
    telemetry.addData("Target Angle", turret.getTargetHorizontalAngleDegrees());
  }

  /**
   * @brief updates everything to do with aiming the turret
   */
  private void updateAiming() {
    /*
    if (tuning) {
      turret.overrideRpm(rpmOverride);
      turret.setVerticalAngle(verticalAngleOverride);

    } else if (turretAimOverride) {
      turret.setHorizontalAngle(horizontalAngleOverride);
      turret.setTargetDistance(distanceOverride);
      return;

    } else if (!limelight.isReadyToNavigate()) {
      turret.setHorizontalAngle(allianceColor.getObeliskAngle());
      return;
    }
     */

    /*
    Pose2D limelightPosition = limelight.getPosition();
    if (limelightPosition != null && turret.isAtTarget()) {
      double targetLimelightHeading =
          robotPosition.getHeading(AngleUnit.DEGREES) + turret.getHorizontalAngleDegrees();
      double limelightHeading = limelightPosition.getHeading(AngleUnit.DEGREES);
      double headingError = targetLimelightHeading - limelightHeading;
      if (Math.abs(headingError) >= 1) turret.changeHorizontalAngleOffsetDegrees(headingError);
    }
     */

    turret.setHorizontalAngle(getRelativeTargetAngle());
    turret.setTargetDistance(getTargetDistance());
  }

  /**
   * @brief updates everything to do with shooting balls out of the turret
   */
  private void updateShooting() {
    if (state != State.SHOOTING) return;
    // ^ only do this logic if emptying

    boolean empty = false;
    switch (sequenceMode) {
      case UNSORTED:
        empty = emptyMagUnsorted();
        break;

      case HALF_SORT:
        empty = emptyMagHalfSorted();
        break;

      case SORTED:
        empty = emptyMagSorted();
        break;
    }

    if (!empty) {
      fillMag();
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
            return;

          case FULL:
            intake.ejectIdle();
            return;

          case SHOOTING:
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
        break;

      case FULL:
        if (spindex.isAtTarget() && intake.isIntaking()) clearIntake();
        break;

      case INTAKING:
        if (spindex.getIsIntakeColorNew() && spindex.isAtTarget()) {
          // ^ if intaking a ball, the spindex is stationary, and a new color of ball is in the
          // intake
          if (spindex.getIntakeColor().isShootable()) {
            // ^ if intake contains a shootable ball
            spindex.storeIntakeColor(); // record the color of the ball taken in
          }
        }

        if (!fillMag()) {
          switchModeToFull();
        }

        break;
    }
  }

  /**
   * @brief switches the scoring system's mode to "full"
   */
  protected void switchModeToFull() {
    state = State.FULL;
    intake.intakeIdle(); // start intake up to keep balls in the mag
    spindex.moveSpindexIdle(spindex.getIndex()); // move spindex to idle position
    turret.activate(); // get ready to shoot at any time
  }

  /**
   * @brief switches the scoring system's mode to "intaking"
   */
  protected void switchModeToIntaking() {
    state = State.INTAKING;
    spindex.moveSpindexIntake(spindex.getColorIndex(BallColor.EMPTY)); // move spindex to empty slot
    intake.intake(); // start the intake spinning
    turret.idle(); // flywheel doesn't need to at full speed
  }

  /**
   * @brief switches the scoring system's mode to "shooting"
   * @note does not do all that needs to be done when switching the mode to shooting
   */
  protected void switchModeToShooting(SequenceMode sequenceMode) {
    state = State.SHOOTING;
    this.sequenceMode = sequenceMode;
    turret.activate(); // just in case; *should* already be active
  }

  /**
   * @brief starts filling the mag with three balls of any color
   * @return true if the mag had empty slots, false if the mag is full
   */
  public boolean fillMag() {
    if (spindex.getColorIndex(BallColor.EMPTY) == NULL) return false;
    // ^ if there are no empty slots in the mag, return false

    switchModeToIntaking();

    return true; // start intaking balls
  }

  /**
   * @brief shoots all balls in the mag, in sequence order if possible
   * @return true if the mag contained at least one ball, false if the mag is empty
   * @note use shootHalfSorted() instead
   */
  @Deprecated
  public boolean shootMag() {
    if (shootSequence()) return true; // try shooting a sequence
    return shootHalfSorted(); // try shooting in any order
  }

  /**
   * @brief shoots all balls in the mag in no particular order
   * @return true if the mag contained at least one ball, false if mag is empty
   */
  public boolean shootUnsorted() {
    int index = NULL;
    for (BallColor color : spindex.getSpindexContents()) { // for each spindex slot
      if (color.isShootable()) { // if slot has a ball in it
        index = spindex.getColorIndex(color); // get index of slot
        break; // don't keep looking for full slots
      }
    }

    if (index == NULL) return false; // if spindex is empty, return false

    switchModeToShooting(SequenceMode.UNSORTED);
    shootIndex(index); // shoot a ball from the non-empty slot

    return true; // we are emptying the mag; return true
  }

  /**
   * @brief starts shooting all balls in the mag in such a way that as many as possible match the
   *     sequence
   * @return true if the mag contained at least one ball, false if the mag is empty
   * @note this method does not require there to be two purples and one green in the mag
   */
  public boolean shootHalfSorted() {
    int index = NULL;
    for (BallColor color : spindex.getSpindexContents()) {
      if (color.isShootable()) {
        // ^ if spindex isn't empty
        index = spindex.getColorIndex(color);
        break; // don't keep looking
      }
    }

    if (!spindex.isIndexValid(index)) return false; // if spindex is empty, return false

    int correctColorIndex = spindex.getColorIndex(ballSequence.getBallColors()[0]);
    if (spindex.isIndexValid(correctColorIndex)) index = correctColorIndex;

    switchModeToShooting(SequenceMode.HALF_SORT);
    sequenceIndex = 0;
    shootIndex(index); // shoot a ball

    return true; // we are emptying the mag; return true
  }

  /**
   * @brief starts shooting a sequence of balls out of the turret
   * @return true if the mag was full, false if the mag isn't full or a sequence is already being
   *     shot
   */
  public boolean shootSequence() {
    int purples = 0;
    int greens = 0;
    for (BallColor color : spindex.getSpindexContents()) {
      if (color == BallColor.PURPLE) purples++;
      if (color == BallColor.GREEN) greens++;
    }
    if ((purples != 2) || (greens != 1)) return false;
    // ^ return false if the wrong number of balls are in the mag
    switchModeToShooting(SequenceMode.SORTED);
    sequenceIndex = 0; // start with the first ball in the sequence
    return emptyMagSorted(); // start emptying mag
  }

  /**
   * @brief gets the state of the scoring system
   * @return the state of the scoring system
   */
  public State getState() {
    return state;
  }

  /**
   * @brief gets the number of full slots in the spindex
   * @return the number of slots in the spindex that contain a ball
   */
  public int getFullSpindexSlots() {
    int toReturn = 0;
    for (BallColor slot : spindex.getSpindexContents()) {
      if (slot.isShootable()) toReturn++;
    }
    return toReturn;
  }

  /**
   * @brief gets if the spindex is empty
   * @return true if no slots contain a ball, false if any slots contain a ball
   */
  public boolean isSpindexEmpty() {
    return getFullSpindexSlots() == 0;
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
   * @brief the internal logic for emptying the mag in a half sorted manner
   * @return true if the mag is being emptied, false if the mag is empty
   */
  protected boolean emptyMagHalfSorted() {
    int indexToShoot;

    try {
      indexToShoot = spindex.getColorIndex(ballSequence.getBallColors()[sequenceIndex]);
    } catch (Exception e) {
      return false; // done shooting sequence
    }

    if (!spindex.isIndexValid(indexToShoot)) {
      // ^ if the next ball in the sequence isn't in the mag
      indexToShoot = spindex.getShootableIndex();
      if (!spindex.isIndexValid(indexToShoot)) {
        return false; // mag empty
      }
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

      if (!spindex.isIndexValid(indexToShoot)) {
        // ^ if the next ball in the sequence isn't in the mag
        indexToShoot = spindex.getShootableIndex();
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

    if (!spindex.isIndexValid(indexToShoot)) return false; // if mag is empty, return false

    shootIndex(indexToShoot);

    if (spindex.isReadyToShoot() && turret.isReadyToShoot()) {
      // ^ if spindex and turret are ready for ball to be lifted
      spindex.liftBall();
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
   * @brief gets the absolute angle from the robot to the target, not accounting for the robots
   *     rotation
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
   * @return the position of the robot, or null if either the target isn't visible or the camera
   *     isn't still
   * @note this returns null under normal operation conditions, be careful
   */
  public Pose2D getRobotPosition() {
    Pose2D limelightPosition = limelight.getPosition();
    if (limelightPosition == null || !turret.isAtTarget()) return null;
    double rotationDegrees = turret.getHorizontalAngleDegrees();
    double x = limelightPosition.getX(DistanceUnit.INCH);
    double y = limelightPosition.getY(DistanceUnit.INCH);
    double heading = limelightPosition.getHeading(AngleUnit.DEGREES) + rotationDegrees;
    return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
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
  @TestOnly
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
   * @brief starts shooting a ball from a given index in the spindex
   * @param index the ball from the spindex to be shot
   */
  private void shootIndex(int index) {
    intake.intakeIdle(); // start intake up to keep balls in the mag
    spindex.moveSpindexShoot(index); // move spindex to correct position
  }
}
