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

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;

public class Spindex {
  /**
   * @brief simple enum to track the spindex's mode / state
   */
  public enum SpindexState {
    UNINITIALIZED,
    IDLE,
    INTAKING,
    SHOOTING,
    LIFTING,
    LIFTED
  }

  /**
   * @brief simple class to store info tied to each spindex slot
   */
  private class SpindexSlot {
    // the color of ball in the spindex slot
    public BallColor color;
    // the position to move the spindex to to intake a ball into this slot
    public final double intakePosition;
    // the position to move the spindex to to shoot a ball from this slot
    public final double shootPosition;
    // the position half-a-slot off from intake position, so balls cannot exit the mag
    public final double idlePosition;

    public SpindexSlot(double intakePosition, double shootPosition, double idlePosition) {
      this.color = BallColor.UNKNOWN;
      this.intakePosition = intakePosition;
      this.shootPosition = shootPosition;
      this.idlePosition = idlePosition;
    }
  }

  /**
   * @brief simple class to control two Axon servos at once
   */
  private class DualServo {
    private final Axon servo1, servo2; // servos
    private double targetPosition; // the angle to move the servos to

    public DualServo(Axon servo1, Axon servo2) {
      this.servo1 = servo1;
      this.servo2 = servo2;
    }

    /**
     * @brief sets the position of the servos
     * @param degrees the angle to turn the servo to
     */
    public void setPosition(double degrees) {
      targetPosition = degrees; // store target position
      // set servo angles:
      servo1.setPosition(targetPosition);
      servo2.setPosition(targetPosition);
    }

    /**
     * @brief gets if both servos are at the target
     * @return true if both servos are at target, false otherwise
     */
    public boolean isAtTarget() {
      return servo1.isAtTarget() && servo2.isAtTarget();
    }

    /**
     * @brief gets the target position of the servos
     * @return the target angle of the servos
     */
    public double getTargetPosition() {
      return targetPosition;
    }
  }

  private final DualServo spinner; // the servos that rotate the divider in the mag
  private final DualServo lifter; // the servos that lift balls into the shooter turret
  private final ColorSensor colorSensor; // the color sensor at the intake
  private final DistanceSensor
      distanceSensor; // the distance sensor at the intake (built into color sensor?)
  private final double lifterRetractedPos =
      0.0; // the position of the lift servo when at rest | TODO: tune
  private final double lifterExtendedPos =
      0.3; // the position of the lift servo when shooting | TODO: tune
  private final SpindexSlot[] spindex = {
    new SpindexSlot(0.0, 120.0, 60.0), // slot 0 | TODO: tune
    new SpindexSlot(120.0, 240.0, 180.0), // slot 1 | TODO: tune
    new SpindexSlot(240.0, 0.00, 300) // slot 2 | TODO: tune
  };

  private SpindexState state; // the current state of the spindex
  private int currentIndex = NULL; // the current index the spindex is at, dependant on the state
  private BallColor intakeColor =
      BallColor.UNKNOWN; // the color of ball in the intake the most recent time checked
  private BallColor oldIntakeColor =
      BallColor.UNKNOWN; // the color of ball in the intake last time checked

  public Spindex(
      Axon spinServo1,
      Axon spinServo2,
      Axon liftServo1,
      Axon liftServo2,
      ColorSensor colorSensor,
      DistanceSensor distanceSensor) {
    this.spinner = new DualServo(spinServo1, spinServo2);
    this.lifter = new DualServo(liftServo1, liftServo2);
    this.colorSensor = colorSensor;
    this.distanceSensor = distanceSensor;
    this.colorSensor.enableLed(true);
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
      for (SpindexSlot slot : spindex) { // for each slot
        slot.color = BallColor.EMPTY; // slot is empty
      }
    }

    // move spindex slot 0 to intake
    spinner.setPosition(spindex[0].intakePosition);
    this.state = SpindexState.INTAKING; // spindex in intaking mode
    this.currentIndex = 0; // spindex at index 0
    this.lifter.setPosition(lifterRetractedPos); // move lifting mechanism to rest position
  }

  /**
   * @brief updates everything to do with the spindex
   */
  public void update() {
    updateIntakeColor();

    // do something different depending on the spindex state / mode
    switch (state) {
      case IDLE: // if the spindex is idle
        if (currentIndex == NULL) break; // nothing can be done without a valid index
        // if the lifter is retracted and the spindex has not been set to the correct position
        if (!isSpindexPosition(spindex[currentIndex].idlePosition) && lifter.isAtTarget()) {
          // move spindex to idle position
          spinner.setPosition(spindex[currentIndex].idlePosition);
        }
        break;

      case INTAKING: // if the spindex is intaking
        if (currentIndex == NULL) break; // nothing can be done without a valid index
        // if the lifter is retracted and the spindex has not been set to the correct position
        if (!isSpindexPosition(spindex[currentIndex].intakePosition) && lifter.isAtTarget()) {
          spinner.setPosition(spindex[currentIndex].intakePosition);
        }
        break;

      case SHOOTING: // if the spindex is shooting
        if (currentIndex == NULL) break; // nothing can be done without a valid index
        // if the lifter is retracted and the spindex has not been set to the correct position
        if (!isSpindexPosition(spindex[currentIndex].shootPosition) && lifter.isAtTarget()) {
          spinner.setPosition(spindex[currentIndex].shootPosition);
        }
        break;

      case LIFTING: // if the spindex is lifting
        if (currentIndex == NULL) break; // nothing can be done without a valid index
        if (lifter.isAtTarget()) { // if lifter is fully extended
          lifter.setPosition(lifterRetractedPos); // retract lifter
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
  }

  /**
   * @param index the spindex index to move to the intake position
   * @brief moves the specified spindex index to its intake position
   */
  public void moveSpindexIntake(int index) {
    if (index < 0 || index >= spindex.length) return; // return on invalid parameters
    // retract lifter
    lifter.setPosition(lifterRetractedPos);
    if (lifter.isAtTarget()) { // if lifter is fully retracted
      // set spindex to new position
      spinner.setPosition(spindex[index].intakePosition);
    }
    currentIndex = index; // set new index
    state = SpindexState.INTAKING; // spindex in intaking mode
  }

  /**
   * @param index the spindex index to move to the shooting position
   * @brief moves the specified spindex index to its shooting position
   */
  public void moveSpindexShoot(int index) {
    if (index < 0 || index >= spindex.length) return; // return on invalid parameters
    // retract lifter
    lifter.setPosition(lifterRetractedPos);
    if (lifter.isAtTarget()) { // if lifter is fully retracted
      // set spindex to new position
      spinner.setPosition(spindex[index].shootPosition);
    }
    currentIndex = index; // set new index
    state = SpindexState.SHOOTING; // spindex in shooting mode
  }

  /**
   * @brief moves the the specified spindex index to its idle position
   * @param index the spindex index to move to the idle position
   * @note idle position is offset by half a slot such that balls cannot exit the spindex
   */
  public void moveSpindexIdle(int index) {
    if (index < 0 || index >= spindex.length) return; // return on invalid parameters
    // retract lifter
    lifter.setPosition(lifterRetractedPos);
    // if the lifter is fully retracted
    if (lifter.isAtTarget()) {
      // set spindex to new position
      spinner.setPosition(spindex[index].idlePosition);
    }
    currentIndex = index; // set new index
    state = SpindexState.IDLE; // spindex in idle mode
  }

  /**
   * @brief stores the color of ball detected in the intake as the color of ball in the spindex
   *     index at the intake position
   * @note uses the stored intake color, call update() to update
   * @return true if the color was stored, false if the spindex wasn't stationary at an intake
   *     position
   */
  public boolean storeIntakeColor() {
    // if spindex isn't stationary at an intake position, return false
    if (state != SpindexState.INTAKING || !isAtTarget()) return false;

    setSpindexIndexColor(currentIndex, intakeColor);
    return true;
  }

  /**
   * @brief starts lifting a ball into the turret
   */
  public boolean liftBall() {
    // if the spindex is not stationary at a valid shooting position, or the lifter is not
    // retracted, we cannot lift a ball
    if (state != SpindexState.SHOOTING || currentIndex == NULL || !isAtTarget()) return false;

    lifter.setPosition(lifterExtendedPos);

    state = SpindexState.LIFTING; // spindex in lifting mode
    return true; // we can lift (and are lifting) a ball
  }

  /**
   * @brief returns if the spindex is at its target position
   * @return true if the spindex is at its target angle and set to the correct target angle for the
   *     mode the spindex is in, false otherwise
   */
  public boolean isAtTarget() {
    boolean isSet = true;
    double targetPosition = spinner.getTargetPosition();
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
    }

    return spinner.isAtTarget() && isSet;
  }

  /**
   * @brief returns if the color of ball in the intake is different from the last reading
   * @return true if the color of ball in the intake changed, false if it didn't
   */
  public boolean getIsIntakeColorNew() {
    return intakeColor == oldIntakeColor;
  }

  /**
   * @brief gets the color of ball in the intake position
   * @return the color of ball in the intake
   * @note the returned value is stored, call update() to update it
   */
  public BallColor getIntakeColor() {
    return intakeColor;
  }

  /**
   * @brief returns an array of the colors of ball in the spindex
   * @return a newly constructed array of the colors of ball in each slot of the spindex
   */
  public BallColor[] getSpindexColor() {
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
   * @brief finds an index in the spindex containing a given color
   * @param color the color wanted
   * @return the index in the spindex where that color is located, or -1 if that color isn't in the
   *     mag
   */
  public int getColorIndex(BallColor color) {
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
    if (index < 0 || index >= spindex.length)
      return BallColor.UNKNOWN; // return on invalid parameters
    return spindex[index].color; // return color of ball at index
  }

  /**
   * @brief gets if the spindex is ready to lift a ball into the turret
   * @return true if the spindex is stationary and in shooting mode, false otherwise
   */
  public boolean isReadyToShoot() {
    return state == SpindexState.SHOOTING && isAtTarget() && currentIndex != NULL;
  }

  /**
   * @brief checks if the lifter is at a specified position
   * @param positionToCheck the position to compare against the lifter's target position
   * @return true if the lifter's target is the same as the specified position, false otherwise
   */
  private boolean isLifterPosition(double positionToCheck) {
    return lifter.getTargetPosition() == positionToCheck;
  }

  /**
   * @brief checks if the target position of the spindex is the same as the position supplied
   * @param positionToCheck the position to compare against the spindex's target position
   * @return true if the supplied position is the same as the spindex's target position, false
   *     otherwise
   */
  private boolean isSpindexPosition(double positionToCheck) {
    return spinner.getTargetPosition() == positionToCheck;
  }

  /**
   * @brief sets the ball color at a specified spindex index
   * @param index the index in the spindex to set the color of
   * @param color the color the spindex index contains
   */
  private void setSpindexIndexColor(int index, BallColor color) {
    spindex[index].color = color;
  }

  /**
   * @brief reads and stores the color of ball that is in the intake
   */
  private void updateIntakeColor() {
    oldIntakeColor = intakeColor; // store old intake ball color
    intakeColor = BallColor.EMPTY; // default to an empty intake

    int red = colorSensor.red();
    int green = colorSensor.green();
    int blue = colorSensor.blue();
    float[] hsv = new float[3];
    Color.RGBToHSV(red * 8, green * 8, blue * 8, hsv);
    float h = hsv[0];
    float s = hsv[1];
    float v = hsv[2];

    if (s > 0.6 && v > 40 && h >= 150 && h <= 170) { // green
      intakeColor = BallColor.GREEN; // intake has a green ball in it
    } else if (s > 0.3 && v > 40 && h >= 220 && h <= 240) { // purple
      intakeColor = BallColor.PURPLE; // intake has a purple ball in it
    } else { // color can't be determined
      double distance = distanceSensor.getDistance(DistanceUnit.INCH);
      if (distance <= 1.0) { // if a ball is in the intake
        intakeColor = BallColor.UNKNOWN; // intake has an unknown ball in it
      }
    }
  }
}
