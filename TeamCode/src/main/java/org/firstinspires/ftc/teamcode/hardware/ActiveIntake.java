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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.CircularAverage;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

@Config
public class ActiveIntake {
  /**
   * Simple enum to capture the state of the intake
   */
  public enum State {
    OFF(0),
    INTAKING(1),
    EJECTING_IDLE(-0.5),
    EJECTING_FAST(-1);

    public final double motorPower;

    State(double motorPower) {
      this.motorPower = motorPower;
    }
  }

  // config vars
  private final static int READING_NUMBER = 10; // number of past readings to average
  private final static double TIMER_DURATION = 1.0;

  // config vars (FTC Dashboard)
  // todo tune these values to be reasonable
  public static double STALL_CURRENT = 5; // current at or above which intake is considered stalled
  public static double READING_INTERVAL = 0.01; // interval (seconds) to read motor current
  public static double AUTO_RESTART_INTERVAL = 1.0; // ^ interval (seconds) to re-command motor
  // (in case of stall and undervoltage shutdown)

  private final DcMotorEx intakeMotor; // the motor driving the intake
  private State state = State.OFF; // state of the intake
  private double current; // last computed average current, in amps
  private final CircularAverage average = new CircularAverage(READING_NUMBER);
  private final ElapsedTime timeSinceAutoRestart; // timer to track time since auto restart
  private final SimpleTimer readingTimer = new SimpleTimer(); // timer for reading interval
  public SimpleTimer timer = new SimpleTimer(TIMER_DURATION);

  public ActiveIntake(DcMotorEx intakeMotor) {
    this.intakeMotor = intakeMotor;
    this.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // brake if zero power
    this.current = 0.0; // start at zero current
    this.timeSinceAutoRestart = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    this.readingTimer.start(READING_INTERVAL);
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

    // re-set motor power periodically in case of stall and undervoltage shutoff
    if (timeSinceAutoRestart.seconds() >= AUTO_RESTART_INTERVAL) {
      timeSinceAutoRestart.reset();
      setMotorPower();
    }
  }

  /**
   * Sets the state of the intake
   *
   * @param state the new state of the intake
   */
  public void setState(State state) {
    if (this.state == state) return;
    this.state = state;
    setMotorPower();
  }

  /**
   * Gets the state of the intake
   *
   * @return the current state of the intake
   */
  public State getState() {
    return state;
  }

  /**
   * @brief gets if the intake is stalled
   * @return true if the current of the intake motor is over the stall current, false otherwise
   * @note does not update motor current readings, call update() to update motor current reading
   */
  public boolean isStalled() {
    return current >= STALL_CURRENT;
  }

  /**
   * Sets the power of the motor according to the current state
   */
  private void setMotorPower() {
    intakeMotor.setPower(this.state.motorPower);
  }
}
