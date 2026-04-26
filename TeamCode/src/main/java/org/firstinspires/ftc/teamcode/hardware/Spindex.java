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

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.motors.UnidirectionalHomableRotator;
import org.firstinspires.ftc.teamcode.hardware.sensors.ColorSensorV3;
import org.firstinspires.ftc.teamcode.hardware.sensors.ElcAbsEncoderAnalog;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

/**
 * Class to control the behavior of a passive-transfer spindexer (spindex). Contains sorting logic,
 * and high-level control of motor movement and other related hardware.
 */
@Config
public class Spindex implements System {
  /** Simple enum to track the spindex's mode / state */
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

  /** Simple class to store info tied to each spindex slot */
  private static class SpindexSlot {
    // the color of ball in the spindex slot
    public BallColor color;
    // the position to move the spindex to in order to intake a ball into this slot
    public final double intakePosition;
    // the position to move the spindex to in order to prepare to shoot a ball from this slot
    public final double shootStartPosition;
    public final double shootEndPosition;

    public SpindexSlot(double intakePosition, double shootStartPosition, double shootEndPosition) {
      this.color = BallColor.UNKNOWN;
      this.intakePosition = intakePosition;
      this.shootStartPosition = shootStartPosition;
      this.shootEndPosition = shootEndPosition;
    }
  }

  /** Simple enum to contain shooting modes */
  public enum ShootingMode {
    FAST(false, "Fast ⚡"),
    SLOW(true, "Slow 🎯");

    public final boolean slow;
    public final String displayText;

    ShootingMode(boolean slow, String displayText) {
      this.slow = slow;
      this.displayText = displayText;
    }

    /**
     * Returns the "toggled" or opposite shooting mode
     *
     * @return the opposite shooting mode
     */
    public ShootingMode toggle() {
      return this == FAST ? SLOW : FAST;
    }
  }

  /** Simple enum to contain sorting modes */
  public enum SortingMode {
    SORTED("Sorted 🔁"),
    FAST("Unsorted 🔀");

    public final String displayText;

    SortingMode(String displayText) {
      this.displayText = displayText;
    }

    /**
     * Returns the "toggled" or opposite sorting mode
     *
     * @return the opposite sorting mode
     */
    public SortingMode toggle() {
      return this == SORTED ? FAST : SORTED;
    }
  }

  // config vars (FTC Dashboard)
  public static boolean PROTECT_PINCH_PINT = true;
  public static double INTAKE_FLAP_CLOSED = 325;
  public static double INTAKE_FLAP_OPEN = 240;
  public static double SHOOT_DELAY_SECONDS = 0.2;
  public static double SLOW_MODE_SLOT_DELAY_SECONDS = 0.05;
  public static ShootingMode shootingMode = ShootingMode.FAST;
  public static SortingMode sortingMode = SortingMode.SORTED;

  SystemReport sensorReport = new SystemReport(SystemStatus.NOMINAL); // latest color sensor report
  SystemReport spinnerReport = new SystemReport(SystemStatus.NOMINAL); // latest spinner report
  SystemReport blockerReport = new SystemReport(SystemStatus.NOMINAL); // latest blocker report
  private final UnidirectionalHomableRotator spinner; // the motor that rotates the mag's divider
  private final Axon intakeBlocker; // servo moving flap to close intake while shooting
  private final ColorSensorV3 colorSensor; // the color sensor at the intake
  private final SpindexSlot[] spindex = {
    // code assumptions: increasing angle shoots
    new SpindexSlot(-70, 258, 18), // slot 0
    new SpindexSlot(50, 18, 138), // slot 1
    new SpindexSlot(170, 138, 258) // slot 2
  };

  private final SimpleTimer shootDelay =
      new SimpleTimer(SHOOT_DELAY_SECONDS); // timer to let ramp drop before shooting
  private final SimpleTimer slotWaitDelay =
      new SimpleTimer(SLOW_MODE_SLOT_DELAY_SECONDS); // timer to wait at each slot while shooting in
  // slow mode
  private SpindexState state = SpindexState.UNINITIALIZED; // the current state of the spindex
  private BallSequence sequence = BallSequence.GPP; // the sequence that is to be shot
  private boolean pinchPointFull = false; // if the pinch point of the intake / spindex holds a ball
  private boolean enabled = true; // if spindex can move, sense, etc.
  private boolean colorSensorEnabled = true; // if color sensor is enabled
  private boolean waitingForTimer = false; // if the spindex is waiting for a timer to finish
  // (specifically when slow shooting, for now)
  private int currentIndex = NULL; // the current index the spindex is at, dependent on the state
  private int sortingOffset = 0; // offset for sorting, basically the number of balls in the ramp
  private boolean pinchBackup = false; // if the pinch point empty backup is active
  private BallColor intakeColor =
      BallColor.UNKNOWN; // the color of ball in the intake the most recent time checked

  public Spindex(HardwareMap hardwareMap) {
    this.spinner =
        new UnidirectionalHomableRotator(
            new MotorEx(hardwareMap, "spindex", Motor.GoBILDA.RPM_117),
            new ElcAbsEncoderAnalog(hardwareMap, "spindexEncoder"),
            0.016,
            0.016,
            0.0,
            5,
            true);
    this.intakeBlocker = new Axon(hardwareMap, "intakeBlocker", "intakeBlockerEncoder");
    this.colorSensor = new ColorSensorV3(hardwareMap, "colorSensor", "intakeBeam");
  }

  /**
   * Initializes the spindex
   *
   * @note call when the "init" button is pressed
   */
  public void init(BallSequence preloadedSequence, boolean isPreloaded, boolean auto) {
    enabled = auto;
    if (enabled) spinner.start();
    if (enabled) intakeBlocker.setPosition(isPreloaded ? INTAKE_FLAP_CLOSED : INTAKE_FLAP_OPEN);

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

  /** Starts up the spindex */
  public void start() {
    if (!enabled) spinner.start();
    enabled = true;
  }

  /** Updates everything to do with the spindex */
  public void update() {
    if (!enabled) return;

    sensorReport = colorSensor.getStatus();
    spinnerReport = spinner.getStatus();
    blockerReport = intakeBlocker.getStatus();

    if (!isIndexValid(currentIndex)) state = SpindexState.UNINITIALIZED; // shouldn't happen

    // do something different depending on the spindex state / mode
    switch (state) {
      case INTAKING: // if the spindex is intaking
        if (PROTECT_PINCH_PINT && pinchPointFull && !pinchBackup)
          break; // wait for ball to clear pinch point

        pinchBackup = false;

        if (isFull()) {
          prepToShootSequence(sequence);
          break;
        } // prepare to shoot if full

        if (isAtTarget()) intakeBlocker.setPosition(INTAKE_FLAP_OPEN); // open intake
        currentIndex = getColorIndex(BallColor.EMPTY);

        turnSpindexNoShoot(spindex[currentIndex].intakePosition); // move spindex to position

        if (isAtTarget() && intakeColor.isShootable()) {
          storeIntakeColor();
        }
        break;

      case SHOOTING_READY: // if the spindex is preparing to shoot
        prepToShootSequence(sequence);
        // move spindex to position if flap closed
        if (intakeBlocker.atTarget()) turnSpindexNoShoot(spindex[currentIndex].shootStartPosition);
        if (!spinner.atTarget()) shootDelay.start();
        break;

      case SHOOTING: // if the spindex is shooting
        intakeBlocker.setPosition(INTAKE_FLAP_CLOSED); // close intake (just in case)

        switch (shootingMode) {
          case FAST:
            if (spinner.atTarget()) { // if spindex is done turning around
              setEmpty();
            }
            break;

          case SLOW:
            if (!waitingForTimer) {
              if (spinner.atTarget()) {
                waitingForTimer = true;
                slotWaitDelay.start(SLOW_MODE_SLOT_DELAY_SECONDS);
                // ^ we need to explicitly give a time because it could be changed at runtime
              }

            } else if (slotWaitDelay.isFinished()) {
              waitingForTimer = false;
              spindex[currentIndex].color = BallColor.EMPTY;
              if (isEmpty()) {
                switchToIntaking();
              } else {
                currentIndex = getIncrementedSlot(currentIndex);
                turnSpindexShoot(spindex[currentIndex].shootEndPosition);
              }
            }
            break;
        }
        break;

      case UNINITIALIZED: // if the spindex is uninitialized
        spinner.setAngle(spindex[0].intakePosition);
        break;
    }

    spinner.update();

    if (colorSensorEnabled && state.checkSensor && isAtTarget()) {
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
      message = "🟥Broken (Color sensor); use backup controls. Is it unplugged?";

    } else if (spinnerStatus == SystemStatus.FALLBACK) {
      status = SystemStatus.FALLBACK;
      message = "🟨Backup (Spinner); will be slower. Is it jammed?";

    } else if (blockerStatus == SystemStatus.FALLBACK) {
      status = SystemStatus.FALLBACK;
      message = "🟨Backup (Intake blocker); will be slower. Is it jammed?";

    } else if (sensorStatus == SystemStatus.FALLBACK) {
      status = SystemStatus.FALLBACK;
      message = "🟨Backup (Color sensor); use backup controls";
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
    if (state == SpindexState.SHOOTING) return; // return if shooting
    intakeBlocker.setPosition(INTAKE_FLAP_CLOSED); // close intake
    currentIndex = getBestStartIndex(sequence); // set new index
    state = SpindexState.SHOOTING_READY; // spindex in shooting mode
    // probably redundant code was removed here, if things are breaking this could be it
  }

  /**
   * Starts shooting all balls in the spindex
   *
   * @note returns if spindex isn't ready to shoot
   */
  public void shoot() {
    if (!isReadyToShoot()) return;
    intakeBlocker.setPosition(INTAKE_FLAP_CLOSED); // close intake (just in case)
    state = SpindexState.SHOOTING;

    switch (shootingMode) {
      case FAST:
        spinner.setDirectionConstraint(
            UnidirectionalHomableRotator.DirectionConstraint.FORWARD_ONLY);
        spinner.manualChangeTargetAngle(400.0);
        // this empties the entire mag; we don't ever need to only partially shoot it
        break;

      case SLOW:
        turnSpindexShoot(spindex[currentIndex].shootEndPosition);
        break;
    }
  }

  /**
   * Cancels any shot the spindexer may be taking
   *
   * @note takes no action whatsoever if state isn't SHOOTING
   * @note intended only as a driver backup
   */
  public void cancelShot() {
    if (state == SpindexState.SHOOTING) {
      // if we are shooting, the spindexer can't be empty
      spinner.setDirectionConstraint(UnidirectionalHomableRotator.DirectionConstraint.REVERSE_ONLY);
      spinner.setAngle(spinner.getNormalizedCurrentAngle());
      waitingForTimer = false;
      state = SpindexState.SHOOTING_READY;
      prepToShootSequence(sequence);
    }
  }

  /**
   * Moves the spindexer so homing will be very fast on next startup
   *
   * @note intended mainly for use at the very end of Auto
   * @note spindex WILL NOT WORK after calling this; call only at the end of auto
   */
  public void prepForShutdown() {
    state = SpindexState.UNINITIALIZED;
    turnSpindexNoShoot(spindex[0].intakePosition);
  }

  /**
   * "Homes" the spindexer (reads the absolute analog encoder)
   *
   * @note only intended as a manual driver backup; normally not needed
   */
  public void reHome() {
    spinner.home();
  }

  /**
   * Disables the spindexer rotator for a short time to allow a stuck artifact to come free
   *
   * @note only intended as a manual driver backup; shouldn't be needed
   */
  public void unJam() {
    spinner.disable();
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
   * Sets the sorting offset, equivalent to the number of balls in the ramp (can be negative)
   *
   * @param sortingOffset the number of balls in the ramp
   */
  public void setSortingOffset(int sortingOffset) {
    this.sortingOffset = sortingOffset;
  }

  /**
   * Gets the current sorting offset being used, basically how many balls are in the ramp
   *
   * @return the current sorting offset
   */
  public int getSortingOffset() {
    return sortingOffset;
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
   * Sets the shooting mode of the spindexer
   *
   * @param newShootingMode the new shooting mode of the spindexer
   * @note prefer using this method over setting directly
   */
  public void setShootingMode(ShootingMode newShootingMode) {
    if (newShootingMode == null || state == SpindexState.SHOOTING) return;
    // todo maybe come up with a more elegant way to handle shooting mode switches while shooting
    shootingMode = newShootingMode;
  }

  /**
   * Gets the shooting mode of the spindexer
   *
   * @return the spindexer's shooting mode
   */
  public ShootingMode getShootingMode() {
    return shootingMode;
  }

  /**
   * Sets the sorting mode of the spindexer
   *
   * @param newSortingMode the new sorting mode of the spindexer
   * @note prefer using this method over setting directly
   */
  public void setSortingMode(SortingMode newSortingMode) {
    if (newSortingMode == null) return;
    sortingMode = newSortingMode;
  }

  /**
   * Gets the sorting mode of the spindexer
   *
   * @return the spindexer's sorting mode
   */
  public SortingMode getSortingMode() {
    return sortingMode;
  }

  /**
   * Returns if the spindex is at its target position (in an "idle" or inactive state)
   *
   * @return true if the spindex is at its target angle and set to the correct target angle for the
   *     mode the spindex is in and the intake flap is at its target, false otherwise
   */
  public boolean isAtTarget() {
    return spinner.atTarget() && intakeBlocker.atTarget();
  }

  /**
   * Sets the color of ball in the intake, and overrides the pinch point as being empty
   *
   * @param color the color of ball in the intake
   * @note intended as a driver backup
   */
  public void manualIntake(BallColor color) {
    if (!state.checkSensor || !color.isShootable()) return;
    pinchBackup = true; // override the pinch point as "empty" for the next loop
    setSpindexIndexColor(currentIndex, color);
  }

  /**
   * Sets if the color sensor is enabled. If it is disabled, setIntakeColor() must be used
   *
   * @param enabled if true, color sensor will be enabled, if false, color sensor will be disabled
   */
  public void setColorSensorEnabled(boolean enabled) {
    colorSensorEnabled = enabled;
  }

  /**
   * Gets if the color sensor is enabled
   *
   * @return true if enabled, false if disabled
   */
  public boolean isColorSensorEnabled() {
    return colorSensorEnabled;
  }

  /**
   * Returns an array of the colors of ball in the spindex
   *
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
   * Manually sets the contents of the spindexer
   *
   * @param contents an array with exactly three items, containing the colors of ball in each slot
   * @note only to be used as a manual backup
   */
  public void setSpindexContents(BallColor[] contents) {
    if (contents == null || contents.length != spindex.length) return;

    for (int i = 0; i < contents.length; i++) {
      spindex[i].color = contents[i];
    }
  }

  /** Sets the spindexer empty, and starts intaking */
  public void setEmpty() {
    for (SpindexSlot slot : spindex) {
      slot.color = BallColor.EMPTY;
    }

    switchToIntaking();
  }

  /**
   * Sets if the pinch point between the intake and spindexer contains an artifact
   *
   * @param pinchPointFull if there is an artifact in the pinch point
   */
  public void setIsPinchPointFull(boolean pinchPointFull) {
    this.pinchPointFull = pinchPointFull;
  }

  /**
   * Gets the current state / mode of the spindex
   *
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
    if (!isIndexValid(index) || sequence == null) return NULL; // check parameters

    BallColor[] sequenceColors = sequence.getBallColors(sortingOffset);
    BallColor[] output = new BallColor[spindex.length];
    int matches = 0;

    for (int i = 0, workingIndex = index, outputIndex = 0; i < spindex.length; i++) {
      BallColor color = spindex[workingIndex].color; // get slot color
      if (color.isShootable()) output[outputIndex++] = color; // add ball to output if shootable
      workingIndex = getIncrementedSlot(workingIndex); // increment slot
    }

    for (int i = 0; i < spindex.length; i++) {
      if (output[i] == sequenceColors[i]) matches++;
    }

    return matches;
  }

  /**
   * Gets which spindex index it would be best to start shooting from, ranked primarily on accuracy
   * and secondarily on speed (distance to slot).
   *
   * @param sequence the sequence of the shot being evaluated
   * @return the index of the best slot to start shooting from
   * @note if sequence is null, this will behave as if not sorting
   */
  public int getBestStartIndex(BallSequence sequence) {
    int bestIndex = 0;
    int bestIndexMatches = Integer.MIN_VALUE;
    double bestIndexDist = Double.POSITIVE_INFINITY;

    for (int i = 0; i < spindex.length; i++) {
      int thisIndexMatches =
          sortingMode == SortingMode.SORTED && sequence != null ? getIndexMatches(i, sequence) : 0;
      double thisIndexDist =
          spinner.getAngleTravel(
              spindex[i].shootStartPosition,
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
   * Finds an index in the spindex containing a given color
   *
   * @param color the color wanted
   * @return the index in the spindex where that color is located, or -1 if that color isn't in the
   *     mag
   */
  public int getColorIndex(BallColor color) {
    if (color == null) return NULL; // return on invalid arguments

    int toReturn = NULL;
    double bestDist = Double.POSITIVE_INFINITY;

    for (int i = 0; i < spindex.length; i++) { // for each spindex slot
      if (spindex[i].color == color) { // if the slot contains the correct color
        double dist =
            spinner.getAngleTravel(
                spindex[i].intakePosition,
                UnidirectionalHomableRotator.DirectionConstraint.REVERSE_ONLY);

        if (dist < bestDist) {
          toReturn = i; // record slot index
          bestDist = dist;
        }
      }
    }

    return toReturn;
  }

  /**
   * Gets if the spindex is full
   *
   * @return true if there are no empty slots in the spindex, false otherwise
   */
  public boolean isFull() {
    for (SpindexSlot slot : spindex) {
      if (!slot.color.isShootable()) return false;
    }

    return true;
  }

  /**
   * Gets if the spindex is empty
   *
   * @return true if there are no full slots in the spindex, false otherwise
   */
  public boolean isEmpty() {
    for (SpindexSlot slot : spindex) {
      if (slot.color.isShootable()) return false;
    }

    return true;
  }

  /**
   * Gets how many full (shootable) slots are in the spindex
   * @return the number of shootable balls in the spindex
   */
  public int fullSlots() {
    int count = 0;
    for (SpindexSlot slot : spindex) {
      if (slot.color.isShootable()) count++;
    }

    return count;
  }

  /**
   * Gets if the spindex is ready to lift a ball into the turret
   *
   * @return true if the spindex is stationary and in shooting mode, false otherwise
   */
  public boolean isReadyToShoot() {
    return state == SpindexState.SHOOTING_READY
        && !isEmpty()
        && isAtTarget()
        && shootDelay.isFinished()
        && isIndexValid(currentIndex);
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
      double shootPos = slot.shootStartPosition;
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

    spinner.setDirectionConstraint(
        spinner.isHomed()
            ? UnidirectionalHomableRotator.DirectionConstraint.REVERSE_ONLY
            : UnidirectionalHomableRotator.DirectionConstraint.NONE);

    spinner.setAngle(target);
  }

  /**
   * Gets the "next" slot in the spindexer, based on a provided slot index
   *
   * @param startingSlot the "current" slot index, or the slot index to increment from
   * @return the "new" slot index, or the incremented slot index (or 0 if given an invalid index)
   */
  private int getIncrementedSlot(int startingSlot) {
    if (!isIndexValid(startingSlot)) return 0;
    return ++startingSlot >= spindex.length ? 0 : startingSlot;
  }

  /**
   * Switches the spindex's mode to intaking
   *
   * @note won't do anything if the spindex is full
   */
  private void switchToIntaking() {
    if (isFull()) return;
    currentIndex = getColorIndex(BallColor.EMPTY);
    state = SpindexState.INTAKING; // spindex back to intaking mode
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
