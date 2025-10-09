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
  private final Servo spinServo; // /< the servo that rotates the divider in the mag
  private final Servo liftServo; // /< the servo that lifts balls into the shooter turret
  private final ColorSensor colorSensor; // /< the color sensor at the intake
  private final DistanceSensor
      distanceSensor; // /< the distance sensor at the intake (built into color sensor?)
  private final double liftServoRestPos =
      0.0; // /< the position of the lift servo when at rest | TODO: tune
  private final double liftServoShootPos =
      0.3; // /< the position of the lift servo when shooting | TODO: tune
  private static final double[] spindexIntake = {0.0, 0.33, 0.66}; // TODO: tune
  private static final double[] spindexShoot = {0.33, 0.66, 0.00}; // TODO: tune
  private final BallColor[] spindexColor = {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
  private final org.firstinspires.ftc.teamcode.utils.SimpleTimer liftServoTimer =
      new SimpleTimer(0.5); // /< timer for lifting ball into flywheel
  private final org.firstinspires.ftc.teamcode.utils.SimpleTimer spinServoTimer =
      new SimpleTimer(0.75); // /< timer for moving spindex

  private int intakeIndex = NULL;
  private int shootIndex = NULL;
  private BallColor intakeColor =
      BallColor.UNKNOWN; // /< the color of ball in the intake the most recent time checked
  private BallColor oldIntakeColor =
      BallColor.UNKNOWN; // /< the color of ball in the intake last time checked

  public Spindex(
      Servo spinServo, Servo liftServo, ColorSensor colorSensor, DistanceSensor distanceSensor) {
    this.spinServo = spinServo;
    this.liftServo = liftServo;
    this.colorSensor = colorSensor;
    this.distanceSensor = distanceSensor;
    this.colorSensor.enableLed(true);
  }

  /**
   * @brief initializes the spindex
   * @note call when the "init" button is pressed
   */
  public void init() {
    this.spinServo.setPosition(spindexIntake[0]); // move spindex slot 0 to intake
    this.intakeIndex = 0;
    this.liftServo.setPosition(liftServoRestPos); // move lifting mechanism to rest position
  }

  /**
   * @brief updates everything to do with the spindex
   */
  public void update() {
    updateIntakeColor();
    if (liftServo.getPosition() == liftServoShootPos && liftServoTimer.isFinished()) {
      liftServo.setPosition(liftServoRestPos); // move lifter back to resting position
    }
  }

  /**
   * @param index the spindex index to move to the intake position
   * @brief moves the specified spindex index to its intake position
   */
  public void moveSpindexIntake(int index) {
    spinServo.setPosition(spindexIntake[index]); // set servo to new position
    spinServoTimer.start(); // start timer for moving spindex
    intakeIndex = index; // set new intake index
    shootIndex = NULL; // spindex isn't at a shooting index
  }

  /**
   * @param index the spindex index to move to the shooting position
   * @brief moves the specified spindex index to its shooting position
   */
  public void moveSpindexShoot(int index) {
    spinServo.setPosition(spindexShoot[index]); // set servo to new position
    spinServoTimer.start(); // start timer for moving spindex
    shootIndex = index; // set new shooting index
    intakeIndex = NULL; // spindex isn't at an intake index
  }

  /**
   * @brief stores the color of ball detected in the intake as the color of ball in the spindex
   *     index at the intake position
   * @note uses the stored intake color, call update() to update
   * @return true if the color was stored, false if the spindex wasn't at an intake position
   */
  public boolean storeIntakeColor() {
    if (intakeIndex == NULL) { // if spindex isn't at a intake position
      return false;

    } else {
      setSpindexIndexColor(intakeIndex, intakeColor);
      return true;
    }
  }

  /**
   * @brief sets the spindex index at the shooting position as empty
   * @return true if the spindex was at a shooting position, false if it wasn't
   */
  public boolean setShootingIndexEmpty() {
    if (shootIndex == NULL) { // if spindex isn't at a shooting position
      return false;

    } else {
      setSpindexIndexColor(shootIndex, BallColor.EMPTY);
      return true;
    }
  }

  /**
   * @brief lifts the lifter mechanism, lifting a ball into the turret
   */
  public void liftBall() {
    liftServo.setPosition(liftServoShootPos);
    liftServoTimer.start();
  }

  /**
   * @brief returns if the spindex is finished moving
   * @return true if the spindex is stationary, false if the spindex is moving
   */
  public boolean getIsSpindexMoved() {
    return spinServoTimer.isFinished();
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
   * @brief returns the spindex index that is at the intake
   * @return the index that is at the intake, or NULL (-1) if the spindex isn't at an intake index
   */
  public int getIntakeIndex() {
    return intakeIndex;
  }

  /**
   * @brief returns the spindex index that is at the turret
   * @return the index that is at the turret, or NULL (-1) if the spindex isn't at a turret index
   */
  public int getShootIndex() {
    return shootIndex;
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
