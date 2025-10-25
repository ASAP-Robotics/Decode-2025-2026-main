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
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

public class Spindex {
  public enum SpindexState {
    INTAKING,
    SHOOTING,
    LIFTING,
    LIFTED,
    UNINITIALIZED
  }

  private final MonodirectionalServo spinServo1; // one servo that rotates the divider in the mag
  private final MonodirectionalServo spinServo2; // other servo that rotates the divider in the mag
  // private final Servo spinServo; // the servo that rotates the divider in the mag
  private final Servo rampServo; // the servo that lifts balls into the shooter turret
  private final ColorSensor colorSensor; // the color sensor at the intake
  private final DistanceSensor
      distanceSensor; // the distance sensor at the intake (built into color sensor?)
  private final double rampServoRetractedPos =
      0.0; // the position of the lift servo when at rest | TODO: tune
  private final double rampServoLiftPos =
      0.3; // the position of the lift servo when shooting | TODO: tune
  private static final double[] spindexIntake = {0.0, 0.33, 0.66}; // TODO: tune
  // position(s) to move the spindex to to insert the lifter lever for a give index TODO: tune
  private static final double[] spindexShootInsert = {0.33, 0.66, 0.00};
  // position(s) to move the spindex to to lift the ball in a given index TODO: tune
  private static final double[] spindexShootLift = {0.66, 0.00, 0.33};
  private final BallColor[] spindexColor = {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
  private final org.firstinspires.ftc.teamcode.utils.SimpleTimer rampServoTimer =
      new SimpleTimer(0.5); // timer for lifting ball into flywheel
  // private final org.firstinspires.ftc.teamcode.utils.SimpleTimer spinServoTimer =
  // new SimpleTimer(0.75); // timer for moving spindex

  private SpindexState state; // the current state of the spindex
  private int currentIndex = NULL; // the current index the spindex is at, dependant on the state
  // private int intakeIndex = NULL;
  // private int shootIndex = NULL;
  private BallColor intakeColor =
      BallColor.UNKNOWN; // the color of ball in the intake the most recent time checked
  private BallColor oldIntakeColor =
      BallColor.UNKNOWN; // the color of ball in the intake last time checked

  public Spindex(
      MonodirectionalServo spinServo1,
      MonodirectionalServo spinServo2,
      Servo rampServo,
      ColorSensor colorSensor,
      DistanceSensor distanceSensor) {
    this.spinServo1 = spinServo1;
    this.spinServo2 = spinServo2;
    this.rampServo = rampServo;
    this.colorSensor = colorSensor;
    this.distanceSensor = distanceSensor;
    this.colorSensor.enableLed(true);
    this.state = SpindexState.UNINITIALIZED;
  }

  /**
   * @brief initializes the spindex
   * @note call when the "init" button is pressed
   */
  public void init() {
    // move spindex slot 0 to intake
    setSpindexPosition(spindexIntake[0]);
    this.state = SpindexState.INTAKING; // spindex in intaking mode
    this.currentIndex = 0; // spindex at index 0
    this.rampServo.setPosition(rampServoRetractedPos); // move lifting mechanism to rest position
  }

  /**
   * @brief updates everything to do with the spindex
   */
  public void update() {
    updateIntakeColor();

    // if the spindex is intaking, and the lifter ramp is out, retract lifter ramp
    if (state != SpindexState.INTAKING && rampServo.getPosition() == rampServoLiftPos) {
      rampServo.setPosition(rampServoRetractedPos); // move lifter back to resting position
    }

    if (currentIndex == NULL) return; // nothing more can be done without a valid index

    // if the spindex is shooting, and the lifter ramp is retracted, extend the lifter ramp
    if (state == SpindexState.SHOOTING
        && rampServo.getPosition() == rampServoRetractedPos
        && getIsSpindexMoved()) {
      rampServo.setPosition(rampServoLiftPos); // move lifter ramp to extended position
    }

    // if the spindex is lifting, and the lifter ramp is extended
    if (state == SpindexState.LIFTING
        && rampServo.getPosition() == rampServoLiftPos
        && rampServoTimer.isFinished()) {
      if (!isSpindexPosition(
          spindexShootLift[
              currentIndex])) { // if the spindex hasn't been set to the correct position yet
        setSpindexPosition(spindexShootLift[currentIndex]); // move divider to force ball up ramp

      } else if (getIsSpindexMoved()) { // if the spindex is stationary at the correct position
        spindexColor[currentIndex] = BallColor.EMPTY; // spindex slot is now empty
        state = SpindexState.LIFTED; // spindex in interim "lifted" mode
      }
    }
  }

  /**
   * @param index the spindex index to move to the intake position
   * @brief moves the specified spindex index to its intake position
   */
  public void moveSpindexIntake(int index) {
    if (index < 0 || index >= spindexColor.length) return; // return on invalid parameters
    setRampPosition(rampServoRetractedPos); // retract lifter ramp
    setSpindexPosition(spindexIntake[index]); // set spindex to new position
    currentIndex = index; // set new index
    state = SpindexState.INTAKING; // spindex in intaking mode
  }

  /**
   * @param index the spindex index to move to the shooting position
   * @brief moves the specified spindex index to its shooting position
   */
  public void moveSpindexShoot(int index) {
    if (index < 0 || index >= spindexColor.length) return; // return on invalid parameters
    setSpindexPosition(spindexShootInsert[index]); // set spindex to new position
    currentIndex = index; // set new index
    state = SpindexState.SHOOTING; // spindex in shooting mode
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
    if (state != SpindexState.INTAKING || !getIsSpindexMoved()) return false;

    setSpindexIndexColor(currentIndex, intakeColor);
    return true;
  }

  /**
   * @brief sets the spindex index at the shooting position as empty
   * @return true if the spindex was finished lifting a ball, false if it wasn't
   * @note DO NOT USE! spindex slots are now automatically set as empty
   */
  @Deprecated
  public boolean setShootingIndexEmpty() {
    if (state != SpindexState.LIFTED) { // if spindex isn't finished lifting a ball
      return false;

    } else {
      setSpindexIndexColor(currentIndex, BallColor.EMPTY);
      return true;
    }
  }

  /**
   * @brief starts lifting a ball into the turret
   * @note most of this operation is actually handled by regular calls to update()
   */
  public boolean liftBall() {
    // if the spindex is not stationary at a valid shooting position, we cannot lift a ball
    if (state != SpindexState.SHOOTING || currentIndex == NULL || !getIsSpindexMoved())
      return false;

    // if the lifter ramp isn't extended, extend it (this shouldn't happen, but just in case)
    if (rampServo.getPosition() != rampServoLiftPos || !rampServoTimer.isFinished()) {
      rampServo.setPosition(rampServoLiftPos);
      rampServoTimer.start();

    } else { // if the lifter ramp is extended, move the divider to force balls up it
      setSpindexPosition(spindexShootLift[currentIndex]); // start moving the spindex
    }

    state = SpindexState.LIFTING; // spindex in lifting mode
    return true; // we can lift (and are lifting) a ball
  }

  /**
   * @brief returns if the spindex is finished moving
   * @return true if the spindex is stationary, false if the spindex is moving
   */
  public boolean getIsSpindexMoved() {
    return spinServo1.isAtTarget() && spinServo2.isAtTarget();
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
   * @return a copy of the internal `spindexColor`
   */
  public BallColor[] getSpindexColor() {
    return spindexColor.clone();
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

    for (int i = 0; i < spindexColor.length; i++) { // for each spindex slot
      if (spindexColor[i] == color) { // if the slot contains the correct color
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
    if (index < 0 || index >= spindexColor.length)
      return BallColor.UNKNOWN; // return on invalid parameters
    return spindexColor[index]; // return color of ball at index
  }

  /**
   * @brief gets if the spindex is ready to lift a ball into the turret
   * @return true if the spindex is stationary and in shooting mode, false otherwise
   */
  public boolean isReadyToShoot() {
    return state == SpindexState.SHOOTING && getIsSpindexMoved() && currentIndex != NULL;
  }

  /**
   * @brief sets the position of the ramp that lifts the ball into the turret
   * @param position the position (degrees) to set the ramp servo to
   */
  private void setRampPosition(double position) {
    rampServo.setPosition(position);
    rampServoTimer.start();
  }

  /**
   * @brief sets the position of the spindex
   * @param position the position (degrees) to set the spindex servos to
   */
  private void setSpindexPosition(double position) {
    spinServo1.setPosition(position);
    spinServo2.setPosition(position);
  }

  /**
   * @brief checks if the target position of the spindex is the same as the position supplied
   * @param positionToCheck the position to compare against the spindex's target position
   * @return true if the supplied position is the same as the spindex's target position, false
   *     otherwise
   */
  private boolean isSpindexPosition(double positionToCheck) {
    return spinServo1.getTargetPosition() == positionToCheck
        && spinServo2.getTargetPosition() == positionToCheck;
  }

  /**
   * @brief sets the ball color at a specified spindex index
   * @param index the index in the spindex to set the color of
   * @param color the color the spindex index contains
   */
  private void setSpindexIndexColor(int index, BallColor color) {
    spindexColor[index] = color;
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
      if (distance <= 2.0) { // if a ball is in the intake
        intakeColor = BallColor.UNKNOWN; // intake has an unknown ball in it
      }
    }
  }
}
