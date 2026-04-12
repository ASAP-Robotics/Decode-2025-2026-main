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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.sensors.BreakBeam;
import org.firstinspires.ftc.teamcode.utils.CircularAverage;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

@Config
public class ActiveIntake {
  public enum PowerLevel {
    OFF(0),
    INTAKING(1),
    REPELLING(-0.4),
    CLEARING(-0.6);

    public final double motorPower;

    PowerLevel(double motorPower) {
      this.motorPower = motorPower;
    }
  }

  /** Simple enum to capture the state of the intake */
  public enum State {
    OFF(PowerLevel.OFF),
    INTAKING(PowerLevel.INTAKING),
    REPELLING(PowerLevel.REPELLING);

    public final PowerLevel powerLevel;

    State(PowerLevel powerLevel) {
      this.powerLevel = powerLevel;
    }
  }

  // config vars
  private static final int READING_NUMBER = 50; // number of past readings to average
  private static final double CLEARING_DURATION = 1.0; // seconds (how long to clear intake for)
  private static final String MOTOR_NAME = "intake";
  private static final String FRONT_SENSOR_NAME = "frontSensor";
  private static final String PINCH_SENSOR_NAME = "pinchSensor";

  // config vars (FTC Dashboard)
  public static double STALL_CURRENT = 6; // current at or above which intake is considered stalled
  public static double READING_INTERVAL = 0.01; // interval (seconds) to read motor current
  public static double AUTO_RESTART_INTERVAL = 1.0; // ^ interval (seconds) to re-command motor
  // (in case of stall and undervoltage shutdown)
  public static double FULL_TIMEOUT = 1.0; // seconds (to remain "full" after sensor sees ball)

  private final DcMotorEx intakeMotor; // the motor driving the intake
  private final BreakBeam frontSensor; // the break beam sensor across the very front of the intake
  private final BreakBeam pinchSensor; // the break beam sensor across the spindexer's pinch point
  private State state = State.OFF; // state of the intake
  private boolean clearing = false; // if the intake is being cleared
  private boolean turnOffWhenEmpty = false; // if the intake should turn off when "empty"
  private double current; // last computed average current, in amps
  private final CircularAverage average = new CircularAverage(READING_NUMBER);
  private final ElapsedTime timeSinceMotorSet; // timer to track time since auto restart
  private final ElapsedTime timeSinceFront; // timer to track time since ball seen by front sensor
  private final ElapsedTime timeSincePinch; // timer to track time since ball seen by pinch sensor
  private final SimpleTimer readingTimer = new SimpleTimer(); // timer for reading interval
  private final SimpleTimer clearingTimer = new SimpleTimer(CLEARING_DURATION);

  public ActiveIntake(HardwareMap hardwareMap) {
    intakeMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
    frontSensor = new BreakBeam(hardwareMap.get(DigitalChannel.class, FRONT_SENSOR_NAME));
    pinchSensor = new BreakBeam(hardwareMap.get(DigitalChannel.class, PINCH_SENSOR_NAME));

    intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // brake if zero power
    current = 0.0; // start at zero current
    timeSinceMotorSet = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    timeSinceFront = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    timeSincePinch = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    readingTimer.start(READING_INTERVAL);
  }

  /**
   * @brief updates the intake, call every loop
   * @note currently, this only updates the current readings for the motor
   */
  public void update() {
    // handle current averaging for stall detection
    if (readingTimer.isFinished()) {
      readingTimer.start(READING_INTERVAL); // restart timer (time could change at runtime)
      double amps = intakeMotor.getCurrent(CurrentUnit.AMPS); // read current
      average.write(amps); // record reading
      current = average.average(); // find average current
    }

    // read break beam sensors
    if (frontSensor.isBroken()) timeSinceFront.reset();
    if (pinchSensor.isBroken()) timeSincePinch.reset();

    // auto-clear if stalled
    if (stalled() && !clearing && state == State.INTAKING) {
      clear();
    }

    // stop clearing when done
    if (clearing && clearingTimer.isFinished()) clearing = false;

    // set motor power
    setMotorPower();
  }

  /**
   * Sets the state of the intake
   *
   * @param state the new state of the intake
   */
  public void setState(State state) {
    if (this.state == state) return;
    this.state = state;
  }

  /**
   * Gets the state of the intake
   *
   * @return the current state of the intake
   */
  public State state() {
    return state;
  }

  /**
   * @brief gets if the intake is stalled
   * @return true if the current of the intake motor is over the stall current, false otherwise
   * @note does not update motor current readings, call update() to update motor current reading
   */
  public boolean stalled() {
    return current >= STALL_CURRENT;
  }

  /** Starts clearing the intake for a (short) period of time */
  public void clear() {
    clearing = true;
    clearingTimer.start();
  }

  /**
   * Gets if the intake is being cleared
   *
   * @return if the intake is being cleared
   */
  public boolean isClearing() {
    return clearing;
  }

  /**
   * Sets if the intake will turn off when empty
   *
   * @param turnOffWhenEmpty if the intake should turn off when empty
   */
  public void setTurnOffWhenEmpty(boolean turnOffWhenEmpty) {
    this.turnOffWhenEmpty = turnOffWhenEmpty;
  }

  /**
   * Gets if the intake is currently configured to turn off when empty
   *
   * @return if the intake turns off when empty
   */
  public boolean turnsOffWhenEmpty() {
    return turnOffWhenEmpty;
  }

  /**
   * Effectively sets the intake as containing a ball for some (short) amount of time
   *
   * @note intended as a driver backup
   */
  public void setContainsBall() {
    timeSinceFront.reset();
  }

  /**
   * Gets if the intake contains a ball
   *
   * @return if a break beam sensor has been broken recently
   */
  public boolean containsBall() {
    return timeSinceFront.seconds() <= FULL_TIMEOUT || timeSincePinch.seconds() <= FULL_TIMEOUT;
  }

  /**
   * Gets if there is a ball in the pinch point of the intake spindexer
   *
   * @return if the pinch point break beam has been broken recently
   */
  public boolean pinchableBall() {
    return timeSincePinch.seconds() <= FULL_TIMEOUT;
  }

  /** Sets the power of the motor according to the current state */
  private void setMotorPower() {
    double targetPower =
        clearing
            ? PowerLevel.CLEARING.motorPower
            : ((turnOffWhenEmpty && !containsBall())
                ? PowerLevel.OFF.motorPower
                : state.powerLevel.motorPower);
    boolean newTargetPower = !MathUtils.areEqual(targetPower, intakeMotor.getPower());

    if (newTargetPower || timeSinceMotorSet.seconds() >= AUTO_RESTART_INTERVAL) {
      timeSinceMotorSet.reset();
      intakeMotor.setPower(targetPower);
    }
  }
}
