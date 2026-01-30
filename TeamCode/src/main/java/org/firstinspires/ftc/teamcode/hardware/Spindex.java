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

import static org.firstinspires.ftc.teamcode.types.Helpers.NULL;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.hardware.motors.UnidirectionalHomableRotator;
import org.firstinspires.ftc.teamcode.hardware.sensors.ColorSensorV3;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;

public class Spindex implements System {
  /**
   * @brief simple enum to track the spindex's mode / state
   */
  public enum SpindexState {
    UNINITIALIZED(false),
    INTAKING(true),
    SHOOTING_READY(false),
    SHOOTING(false);

    public final boolean checkSensor;

    SpindexState(boolean checkSensor) {
      this.checkSensor = checkSensor;
    }
  }

  /**
   * @brief simple class to store info tied to each spindex slot
   */
  private static class SpindexSlot {
    // the color of ball in the spindex slot
    public BallColor color;
    // the position to move the spindex to to intake a ball into this slot
    public final double intakePosition;
    // the position to move the spindex to to prepare to shoot a ball from this slot
    public final double shootPosition;

    public SpindexSlot(double intakePosition, double shootPosition) {
      this.color = BallColor.UNKNOWN;
      this.intakePosition = intakePosition;
      this.shootPosition = shootPosition;
    }
  }

  private static final double INTAKE_FLAP_CLOSED = 335;
  private static final double INTAKE_FLAP_OPEN = 248;

  SystemReport sensorReport = new SystemReport(SystemStatus.NOMINAL); // latest color sensor report
  SystemReport spinnerReport = new SystemReport(SystemStatus.NOMINAL); // latest spinner report
  SystemReport blockerReport = new SystemReport(SystemStatus.NOMINAL); // latest blocker report
  private final UnidirectionalHomableRotator spinner; // the motor that rotates the mag's divider
  private final Axon intakeBlocker; // servo moving flap to close intake while shooting
  private final ColorSensorV3 colorSensor; // the color sensor at the intake
  private final SpindexSlot[] spindex = {
    // TODO: fine tune
    // code assumptions: slots with higher index have larger angles, and that increasing angle
    // shoots
    new SpindexSlot(8, 30), // slot 0
    new SpindexSlot(128, 150), // slot 1
    new SpindexSlot(248, 270) // slot 2
  };

  private SpindexState state = SpindexState.UNINITIALIZED; // the current state of the spindex
  private BallSequence sequence = BallSequence.GPP; // the sequence that is to be shot
  private int currentIndex = NULL; // the current index the spindex is at, dependant on the state
  private BallColor intakeColor =
      BallColor.UNKNOWN; // the color of ball in the intake the most recent time checked
  private BallColor oldIntakeColor =
      BallColor.UNKNOWN; // the color of ball in the intake last time checked

  public Spindex(
      Motor spinner, TouchSensor homingSwitch, Axon intakeBlocker, ColorSensorV3 colorSensor) {
    this.spinner =
        new UnidirectionalHomableRotator(spinner, homingSwitch, 0.1, 0.05, 0.002, 1, true);
    this.intakeBlocker = intakeBlocker;
    this.colorSensor = colorSensor;
  }

  /**
   * Initializes the spindex
   *
   * @note call when the "init" button is pressed
   */
  public void init(BallSequence preloadedSequence, boolean isPreloaded) {
    spinner.setAngle(0); // temporary, to avoid sudden motion after homing
    spinner.home();
    intakeBlocker.setPosition(isPreloaded ? INTAKE_FLAP_CLOSED : INTAKE_FLAP_OPEN);

    if (isPreloaded) { // if the spindex is preloaded
      for (int i = 0; i < spindex.length; i++) { // for each spindex slot
        spindex[i].color = preloadedSequence.getBallColors()[i]; // set contained ball color
      }

    } else { // if the spindex is empty
      for (SpindexSlot spindexSlot : spindex) { // for each slot
        spindexSlot.color = BallColor.EMPTY; // slot is empty
      }
    }

    state = isPreloaded ? SpindexState.SHOOTING_READY : SpindexState.INTAKING;
    currentIndex = 0; // spindex at index 0
  }

  /** Updates everything to do with the spindex */
  public void update() {
    sensorReport = colorSensor.getStatus();
    spinnerReport = spinner.getStatus();
    blockerReport = intakeBlocker.getStatus();

    if (!isIndexValid(currentIndex)) state = SpindexState.UNINITIALIZED; // shouldn't happen

    // do something different depending on the spindex state / mode
    // direction constraints assume that forwards shoots, backwards doesn't
    switch (state) {
      case INTAKING: // if the spindex is intaking
        if (!isIndexValid(getColorIndex(BallColor.EMPTY))) {
          prepSlotForShoot(getBestStartIndex(sequence));
          break;
        } // prepare to shoot if full

        intakeBlocker.setPosition(INTAKE_FLAP_OPEN); // open intake
        currentIndex = getColorIndex(BallColor.EMPTY);
        // move spindex to position if flap open
        if (intakeBlocker.atTarget()) turnSpindexNoShoot(spindex[currentIndex].intakePosition);

        if (getIsIntakeColorNew() && isAtTarget() && intakeColor.isShootable()) {
          storeIntakeColor();
        }
        break;

      case SHOOTING_READY: // if the spindex is preparing to shoot
        intakeBlocker.setPosition(INTAKE_FLAP_CLOSED); // close intake (just in case)
        // move spindex to position if flap closed
        if (intakeBlocker.atTarget()) turnSpindexNoShoot(spindex[currentIndex].shootPosition);
        break;

      case SHOOTING: // if the spindex is shooting
        intakeBlocker.setPosition(INTAKE_FLAP_CLOSED); // close intake (just in case)
        if (spinner.atTarget()) { // if spindex is done turning around
          for (SpindexSlot slot : spindex) { // spindex is now empty
            slot.color = BallColor.EMPTY;
          }
          currentIndex = getColorIndex(BallColor.EMPTY);
          state = SpindexState.INTAKING; // spindex back to intaking mode
        }
        break;

      case UNINITIALIZED: // if the spindex is uninitialized
        spinner.setAngle(0);
        break;
    }

    spinner.update();

    oldIntakeColor = intakeColor; // store old intake color
    if (state.checkSensor && isAtTarget()) {
      colorSensor.update();
      intakeColor = colorSensor.getColor(); // update intake color
    } else {
      intakeColor = BallColor.INVALID;
    }
  }

  public SystemReport getStatus() {
    SystemStatus status = SystemStatus.NOMINAL;
    SystemStatus sensorStatus = sensorReport.status;
    SystemStatus spinnerStatus = spinnerReport.status;
    SystemStatus blockerStatus = blockerReport.status;
    String message = "🟩Normal";

    if (spinnerStatus == SystemStatus.INOPERABLE) {
      status = SystemStatus.INOPERABLE;
      message = "🟥Broken (Spinner). Is it jammed?";

    } else if (blockerStatus == SystemStatus.INOPERABLE) {
      status = SystemStatus.INOPERABLE;
      message = "🟥Broken (Intake blocker). Is it jammed?";

    } else if (sensorStatus == SystemStatus.INOPERABLE) {
      status = SystemStatus.INOPERABLE;
      message = "🟥Broken (Color sensor); use backups controls. Is it unplugged?";

    } else if (spinnerStatus == SystemStatus.FALLBACK) {
      status = SystemStatus.FALLBACK;
      message = "🟨Backup (Spinner); performance will be degraded. Is it jammed?";

    } else if (blockerStatus == SystemStatus.FALLBACK) {
      status = SystemStatus.FALLBACK;
      message = "🟨Backup (Intake blocker); performance will be degraded. Is it jammed?";

    } else if (sensorStatus == SystemStatus.FALLBACK) {
      status = SystemStatus.FALLBACK;
      message = "🟨Backup (Color sensor); use backups controls";
    }

    return new SystemReport(status, message);
  }

  /**
   * Prepares the spindex to shoot the given sequence
   *
   * @param sequence the sequence to prepare to shoot
   * @note should fail gracefully and silently if passed null, but avoid doing so
   */
  public void prepToShootSequence(BallSequence sequence) {
    prepSlotForShoot(getBestStartIndex(sequence));
  }

  /**
   * Moves the specified spindex index to its shooting position (ready to shoot)
   *
   * @param index the spindex index to move to the shooting position
   */
  protected void prepSlotForShoot(int index) {
    if (!isIndexValid(index) || state == SpindexState.SHOOTING)
      return; // return on invalid parameters
    intakeBlocker.setPosition(INTAKE_FLAP_CLOSED); // close intake
    currentIndex = index; // set new index
    state = SpindexState.SHOOTING_READY; // spindex in shooting mode
  }

  /**
   * Starts shooting all balls in the spindex
   *
   * @note returns if spindex isn't ready to shoot
   */
  public void shoot() {
    if (state != SpindexState.SHOOTING_READY || !spinner.atTarget() || !intakeBlocker.atTarget())
      return;
    intakeBlocker.setPosition(INTAKE_FLAP_CLOSED); // close intake (just in case)
    state = SpindexState.SHOOTING;
    spinner.setDirectionConstraint(UnidirectionalHomableRotator.DirectionConstraint.FORWARD_ONLY);
    spinner.changeTargetAngle(360.0);
  }

  /**
   * Moves the spindexer so homing will be very fast on next startup
   *
   * @note intended mainly for use at the very end of Auto
   * @note spindex WILL NOT WORK after calling this; call only at the end of auto
   */
  public void prepForShutdown() {
    state = SpindexState.UNINITIALIZED;
    turnSpindexNoShoot(-10);
  }

  /**
   * Homes the spindexer
   *
   * @note only intended as a manual driver backup; normally not needed or helpful
   */
  public void reHome() {
    spinner.home();
  }

  /**
   * Stores the color of ball detected in the intake as the color of ball in the spindex index at
   * the intake position
   *
   * @note uses the stored intake color, call update() to update
   */
  protected void storeIntakeColor() {
    // if spindex isn't stationary at an intake position, return false
    if (state != SpindexState.INTAKING || !isAtTarget() || !intakeColor.isShootable()) return;

    setSpindexIndexColor(currentIndex, intakeColor);
  }

  /**
   * Sets the ball sequence that the spindex tries to shoot
   *
   * @param sequence the sequence to shoot
   */
  public void setSequence(BallSequence sequence) {
    this.sequence = sequence;
  }

  /**
   * Returns if the spindex is at its target position (in a "idle" or inactive state)
   *
   * @return true if the spindex is at its target angle and set to the correct target angle for the
   *     mode the spindex is in and the intake flap is at its target, false otherwise
   */
  public boolean isAtTarget() {
    return spinner.atTarget() && intakeBlocker.atTarget();
  }

  /**
   * Returns if the color of ball in the intake is different from the last reading
   *
   * @return true if the color of ball in the intake changed, false if it didn't
   */
  protected boolean getIsIntakeColorNew() {
    return intakeColor != oldIntakeColor;
  }

  /**
   * @brief gets the color of ball in the intake position
   * @return the color of ball in the intake
   * @note the returned value is stored, call update() to update it
   */
  public BallColor getIntakeColor() {
    return intakeColor;
  }

  public void setIntakeColor(BallColor color) {
    setSpindexIndexColor(currentIndex, color);
  }

  /**
   * @brief returns an array of the colors of ball in the spindex
   * @return a newly constructed array of the colors of ball in each slot of the spindex
   */
  public BallColor[] getSpindexContents() {
    BallColor[] toReturn = new BallColor[spindex.length];
    for (int i = 0; i < spindex.length; i++) {
      toReturn[i] = spindex[i].color;
    }
    return toReturn;
  }

  /**
   * Returns the spindex index that is currently active
   *
   * @return the index that is at the intake, or NULL (-1) if the spindex isn't at an intake index
   * @note what the "active" index means depends on state / mode (e.g. intaking vs shooting)
   */
  public int getIndex() {
    return currentIndex;
  }

  /**
   * @brief gets the current state / mode of the spindex
   * @return the current state / mode of the spindex
   */
  public SpindexState getState() {
    return state;
  }

  /**
   * Gets how closely shooting from a given index will match a given ball sequence
   *
   * @param index the index from the spindex to get the number of sequence matches from
   * @param sequence the sequence to compare against
   * @return the number of matching colors in the sequence and the spindex from the given index
   * @note will return -1 on invalid parameters
   */
  public int getIndexMatches(int index, BallSequence sequence) {
    if (index >= spindex.length || index < 0 || sequence == null) return NULL;

    int matches = 0;

    for (BallColor color : sequence.getBallColors()) {
      if (spindex[index].color == color) matches++;
      if (++index >= spindex.length) index = 0;
    }

    return matches;
  }

  /**
   * Gets which spindex index it would be best to start shooting from, ranked primarily on accuracy
   * and secondarily on speed (distance to slot).
   *
   * @param sequence the sequence of the shot being evaluated
   * @return the index of the best slot to start shooting from
   * @note if sequence is null, this will return 0
   */
  public int getBestStartIndex(BallSequence sequence) {
    if (sequence == null) return 0; // trying to fail in a non-catastrophic way
    int bestIndex = 0;
    int bestIndexMatches = Integer.MIN_VALUE;
    double bestIndexDist = Double.POSITIVE_INFINITY;

    for (int i = 0; i < spindex.length; i++) {
      int thisIndexMatches = getIndexMatches(i, sequence);
      double thisIndexDist =
          spinner.getAngleTravel(
              spindex[i].shootPosition,
              UnidirectionalHomableRotator.DirectionConstraint.REVERSE_ONLY);

      if (thisIndexMatches > bestIndexMatches
          || (thisIndexMatches == bestIndexMatches && thisIndexDist < bestIndexDist)) {
        bestIndex = i;
        bestIndexMatches = thisIndexMatches;
        bestIndexDist = thisIndexDist;
      }
    }

    return bestIndex;
  }

  /**
   * Gets an index containing a shootable ball
   *
   * @return the index of a slot containing a ball, or -1 if the spindex is empty
   */
  public int getShootableIndex() {
    int toReturn = NULL;

    for (SpindexSlot slot : spindex) {
      if (slot.color.isShootable()) {
        toReturn = getColorIndex(slot.color);
        break;
      }
    }

    return toReturn;
  }

  /**
   * Finds an index in the spindex containing a given color
   *
   * @param color the color wanted
   * @return the index in the spindex where that color is located, or -1 if that color isn't in the
   *     mag
   */
  public int getColorIndex(BallColor color) {
    if (color == null) return NULL; // return on invalid arguments

    int toReturn = NULL;

    for (int i = 0; i < spindex.length; i++) { // for each spindex slot
      if (spindex[i].color == color) { // if the slot contains the correct color
        toReturn = i; // record slot index
        break; // don't keep looking for a slot
      }
    }

    return toReturn;
  }

  /**
   * Gets the color of ball at a given index in the spindex
   *
   * @param index the index to get the color of contained ball of
   * @return the color of ball in the given index in the spindex
   */
  public BallColor getIndexColor(int index) {
    if (!isIndexValid(index)) return BallColor.INVALID; // return on invalid parameters
    return spindex[index].color; // return color of ball at index
  }

  /**
   * Gets if the spindex is ready to lift a ball into the turret
   *
   * @return true if the spindex is stationary and in shooting mode, false otherwise
   */
  public boolean isReadyToShoot() {
    return state == SpindexState.SHOOTING_READY && isAtTarget() && isIndexValid(currentIndex);
  }

  /**
   * Gets if a spindex index is valid
   *
   * @param index the spindex index to check
   * @return true if the index is contained in the spindex, false if the index is invalid
   * @note this method should always return false if passed -1 (NULL)
   */
  public boolean isIndexValid(int index) {
    return index >= 0 && index < spindex.length;
  }

  /**
   * Checks if moving to a target position will (inadvertently) shoot a ball
   *
   * @param targetPosition the position being moved to
   * @return true if the movement will shoot a ball, false otherwise
   */
  protected boolean willMovementShootBall(double targetPosition) {
    double currentPosition =
        spinner.atTarget()
            ? spinner.getNormalizedTargetAngle()
            : spinner.getNormalizedCurrentAngle();

    for (SpindexSlot slot : spindex) {
      double shootPos = slot.shootPosition;
      // if slot contains a ball that will be inadvertently shot
      if (slot.color.isShootable() && shootPos <= targetPosition && shootPos >= currentPosition) {
        return true;
      }
    }

    return false;
  }

  /**
   * Turns the spindex to an angle in such a way that a ball could be shot
   *
   * @param target the angle to turn to
   */
  private void turnSpindexShoot(double target) {
    spinner.setDirectionConstraint(UnidirectionalHomableRotator.DirectionConstraint.FORWARD_ONLY);
    spinner.setAngle(target);
  }

  /**
   * Turns the spindex to an angle in such a way that balls aren't shot
   *
   * @param target the angle to turn to
   */
  private void turnSpindexNoShoot(double target) {
    spinner.setDirectionConstraint(UnidirectionalHomableRotator.DirectionConstraint.REVERSE_ONLY);
    spinner.setAngle(target);
  }

  /**
   * Checks if the target position of the spindex is the same as the position supplied
   *
   * @param positionToCheck the position to compare against the spindex's target position
   * @return true if the supplied position is the same as the spindex's target position, false
   *     otherwise
   */
  private boolean isSpindexPosition(double positionToCheck) {
    return spinner.getNormalizedCurrentAngle() == positionToCheck;
  }

  /**
   * Sets the ball color at a specified spindex index
   *
   * @param index the index in the spindex to set the color of
   * @param color the color the spindex index contains
   */
  private void setSpindexIndexColor(int index, BallColor color) {
    spindex[index].color = color;
  }
}
