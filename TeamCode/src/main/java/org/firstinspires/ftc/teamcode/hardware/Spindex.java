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

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.hardware.servos.UnidirectionalAxon;
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
    IDLE(false),
    INTAKING(true),
    SHOOTING(false),
    LIFTING(false),
    LIFTED(false);

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
    // the position to move the spindex to to force a ball up the ramp
    public final double liftPosition;
    // the position half-a-slot off from intake position, so balls cannot exit the mag
    public final double idlePosition;

    public SpindexSlot(
        double intakePosition, double shootPosition, double liftPosition, double idlePosition) {
      this.color = BallColor.UNKNOWN;
      this.intakePosition = intakePosition;
      this.shootPosition = shootPosition;
      this.liftPosition = liftPosition;
      this.idlePosition = idlePosition;
    }
  }

  SystemReport sensorReport = new SystemReport(SystemStatus.NOMINAL); // latest color sensor report
  SystemReport spinnerReport = new SystemReport(SystemStatus.NOMINAL); // latest spinner report
  private final UnidirectionalAxon spinner; // the servos that rotate the divider in the mag
  private final ColorSensorV3 colorSensor; // the color sensor at the intake
  private final SpindexSlot[] spindex = {
    // TODO: retune after rework
    new SpindexSlot(70, 333, 333, 333), // slot 0
    new SpindexSlot(203, 100, 100, 100), // slot 1
    new SpindexSlot(333, 200, 200, 200) // slot 2
  };

  private SpindexState state; // the current state of the spindex
  private int currentIndex = NULL; // the current index the spindex is at, dependant on the state
  private BallColor intakeColor =
      BallColor.UNKNOWN; // the color of ball in the intake the most recent time checked
  private BallColor oldIntakeColor =
      BallColor.UNKNOWN; // the color of ball in the intake last time checked

  public Spindex(CRServo spinServo, AnalogInput spinEncoder, ColorSensorV3 colorSensor) {
    this.spinner = new UnidirectionalAxon(spinServo, spinEncoder);
    this.colorSensor = colorSensor;
    this.state = SpindexState.UNINITIALIZED;
  }

  /**
   * @brief initializes the spindex
   * @note call when the "init" button is pressed
   */
  public void init(BallSequence preloadedSequence, boolean isPreloaded) {
    if (isPreloaded) { // if the spindex is preloaded
      for (int i = 0; i < spindex.length; i++) { // for each spindex slot
        spindex[i].color = preloadedSequence.getBallColors()[i]; // set contained ball color
      }

    } else { // if the spindex is empty
      for (SpindexSlot spindexSlot : spindex) { // for each slot
        spindexSlot.color = BallColor.EMPTY; // slot is empty
      }
    }

    state = isPreloaded ? SpindexState.IDLE : SpindexState.INTAKING;
    currentIndex = 0; // spindex at index 0
  }

  /**
   * @brief updates everything to do with the spindex
   */
  public void update() {
    sensorReport = colorSensor.getStatus();
    spinnerReport = spinner.getStatus();

    if (!isIndexValid(currentIndex)) state = SpindexState.UNINITIALIZED; // shouldn't happen

    // do something different depending on the spindex state / mode
    // direction constraints assume that forwards shoots, backwards doesn't
    switch (state) {
      case IDLE: // if the spindex is idle
        turnSpindexNoShoot(spindex[currentIndex].idlePosition);
        break;

      case INTAKING: // if the spindex is intaking
        turnSpindexNoShoot(spindex[currentIndex].intakePosition);
        break;

      case SHOOTING: // if the spindex is shooting
        turnSpindexNoShoot(spindex[currentIndex].shootPosition);
        break;

      case LIFTING: // if the spindex is lifting
        turnSpindexShoot(spindex[currentIndex].liftPosition);
        if (spinner.isAtTarget()) { // if spindex is at lift position
          spindex[currentIndex].color = BallColor.EMPTY; // spindex slot is now empty
          state = SpindexState.LIFTED; // spindex in interim "lifted" mode
        }
        break;

      case LIFTED: // if the spindex is lifted
        // nothing needs to be done
        break;

      case UNINITIALIZED: // if the spindex is uninitialized
        // nothing needs to be done
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
    String message = "🟩Normal";

    if (spinnerStatus == SystemStatus.INOPERABLE) {
      status = SystemStatus.INOPERABLE;
      message = "🟥Broken (Spinner)";

    } else if (sensorStatus == SystemStatus.INOPERABLE) {
      status = SystemStatus.INOPERABLE;
      message = "🟥Broken (Color sensor); use backups controls";

    } else if (spinnerStatus == SystemStatus.FALLBACK) {
      status = SystemStatus.FALLBACK;
      message = "🟨Backup (Spinner); performance will be degraded";

    } else if (sensorStatus == SystemStatus.FALLBACK) {
      status = SystemStatus.FALLBACK;
      message = "🟨Backup (Color sensor); use backups controls";
    }

    return new SystemReport(status, message);
  }

  /**
   * @param index the spindex index to move to the intake position
   * @brief moves the specified spindex index to its intake position
   */
  public void moveSpindexIntake(int index) {
    if (!isIndexValid(index)) return; // return on invalid parameters
    spinner.setTargetRotation(spindex[index].intakePosition);
    currentIndex = index; // set new index
    state = SpindexState.INTAKING; // spindex in intaking mode
  }

  /**
   * @param index the spindex index to move to the shooting position
   * @brief moves the specified spindex index to its shooting position
   */
  public void moveSpindexShoot(int index) {
    if (!isIndexValid(index) || state == SpindexState.SHOOTING || state == SpindexState.LIFTING)
      return; // return on invalid parameters
    // set spindex to new position
    spinner.setTargetRotation(spindex[index].shootPosition);
    currentIndex = index; // set new index
    state = SpindexState.SHOOTING; // spindex in shooting mode
  }

  /**
   * @brief moves the the specified spindex index to its idle position
   * @param index the spindex index to move to the idle position
   * @note idle position is offset by half a slot such that balls cannot exit the spindex
   */
  public void moveSpindexIdle(int index) {
    if (!isIndexValid(index)) return; // return on invalid parameters
    // set spindex to new position
    spinner.setTargetRotation(spindex[index].idlePosition);
    currentIndex = index; // set new index
    state = SpindexState.IDLE; // spindex in idle mode
  }

  /**
   * @brief stores the color of ball detected in the intake as the color of ball in the spindex
   *     index at the intake position
   * @note uses the stored intake color, call update() to update
   */
  public void storeIntakeColor() {
    // if spindex isn't stationary at an intake position, return false
    if (state != SpindexState.INTAKING || !isAtTarget() || !intakeColor.isShootable()) return;

    setSpindexIndexColor(currentIndex, intakeColor);
  }

  /**
   * @brief starts lifting a ball into the turret
   * @note only starts lifting after the next call to update()
   */
  public void liftBall() {
    // if the spindex is not stationary at a valid shooting position, we cannot lift a ball
    if (state != SpindexState.SHOOTING || currentIndex == NULL) return;
    state = SpindexState.LIFTING; // spindex in lifting mode
  }

  /**
   * @brief returns if the spindex is at its target position (in a "idle" or inactive state)
   * @return true if the spindex is at its target angle and set to the correct target angle for the
   *     mode the spindex is in and the lifter is at its target, false otherwise
   */
  public boolean isAtTarget() {
    boolean isSet = true;
    double targetPosition = spinner.getNormalizedTargetRotation();
    switch (state) {
      case IDLE:
        if (targetPosition != spindex[currentIndex].idlePosition) isSet = false;
        break;

      case INTAKING:
        if (targetPosition != spindex[currentIndex].intakePosition) isSet = false;
        break;

      case SHOOTING:
        if (targetPosition != spindex[currentIndex].shootPosition) isSet = false;
        break;

      case LIFTING:
        isSet = false;
        break;
    }

    return spinner.isAtTarget() && isSet;
  }

  /**
   * @brief returns if the color of ball in the intake is different from the last reading
   * @return true if the color of ball in the intake changed, false if it didn't
   */
  public boolean getIsIntakeColorNew() {
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
   * @brief returns the spindex index that is currently active
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
   * @brief gets an index containing a shootable ball
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
   * @brief finds an index in the spindex containing a given color
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
   * @brief gets the color of ball at a given index in the spindex
   * @param index the index to get the color of contained ball of
   * @return the color of ball in the given index in the spindex
   */
  public BallColor getIndexColor(int index) {
    if (!isIndexValid(index)) return BallColor.INVALID; // return on invalid parameters
    return spindex[index].color; // return color of ball at index
  }

  /**
   * @brief gets if the spindex is ready to lift a ball into the turret
   * @return true if the spindex is stationary and in shooting mode, false otherwise
   */
  public boolean isReadyToShoot() {
    return state == SpindexState.SHOOTING && isAtTarget() && isIndexValid(currentIndex);
  }

  /**
   * @brief gets if a spindex index is valid
   * @param index the spindex index to check
   * @return true if the index is contained in the spindex, false if the index is invalid
   * @note this method should always return false if passed -1 (NULL)
   */
  public boolean isIndexValid(int index) {
    return index >= 0 && index < spindex.length;
  }

  /**
   * @brief checks if moving to a target position will (inadvertently) shoot a ball
   * @param targetPosition the position being moved to
   * @return true if the movement will shoot a ball, false otherwise
   */
  protected boolean willMovementShootBall(double targetPosition) {
    double currentPosition =
        spinner.isAtTarget() ? spinner.getNormalizedTargetRotation() : spinner.getCurrentRotation();

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
   * @brief turns the spindex to an angle in such a way that a ball could be shot
   * @param target the angle to turn to
   */
  private void turnSpindexShoot(double target) {
    spinner.setDirectionConstraint(UnidirectionalAxon.DirectionConstraint.FORWARD_ONLY);
    spinner.setTargetRotation(target);
  }

  /**
   * @brief turns the spindex to an angle in such a way that balls aren't shot
   * @param target the angle to turn to
   */
  private void turnSpindexNoShoot(double target) {
    spinner.setDirectionConstraint(
        willMovementShootBall(target)
            ? UnidirectionalAxon.DirectionConstraint.REVERSE_ONLY
            : UnidirectionalAxon.DirectionConstraint.NONE);
    spinner.setTargetRotation(target);
  }

  /**
   * @brief checks if the target position of the spindex is the same as the position supplied
   * @param positionToCheck the position to compare against the spindex's target position
   * @return true if the supplied position is the same as the spindex's target position, false
   *     otherwise
   */
  private boolean isSpindexPosition(double positionToCheck) {
    return spinner.getNormalizedTargetRotation() == positionToCheck;
  }

  /**
   * @brief sets the ball color at a specified spindex index
   * @param index the index in the spindex to set the color of
   * @param color the color the spindex index contains
   */
  private void setSpindexIndexColor(int index, BallColor color) {
    spindex[index].color = color;
  }
}
