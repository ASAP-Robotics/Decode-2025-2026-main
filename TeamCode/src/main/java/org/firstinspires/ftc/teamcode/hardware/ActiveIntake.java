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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.LinkedList;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

public class ActiveIntake {
  private class CurrentReading {
    public double current;
    public double timeStamp;

    public CurrentReading(double current, double timeStamp) {
      this.current = current;
      this.timeStamp = timeStamp;
    }
  }

  private final DcMotorEx intakeMotor; // the motor driving the intake
  private boolean idling = true; // if the intake is idling
  private boolean intaking = false; // if the intake is intaking
  private boolean ejecting = false; // if the intake is ejecting
  private boolean ballIn = false; // if there is a ball in the intake
  private double current; // last computed average current, in amps
  private double stallCurrent; // current at or above which intake is considered stalled
  private double readingTime; // time span (seconds) to take current reading average over
  private ElapsedTime timeSinceStart; // timer to track time since object creation
  private LinkedList<CurrentReading> currentReadings; // list of current readings and timestamps
  public org.firstinspires.ftc.teamcode.utils.SimpleTimer timer = new SimpleTimer(1);

  public ActiveIntake(DcMotorEx intakeMotor) {
    this(intakeMotor, 1, 5);
  }

  public ActiveIntake(DcMotorEx intakeMotor, double readingTime, double stallCurrent) {
    this.intakeMotor = intakeMotor;
    this.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); // spin forwards
    this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // brake if zero power
    this.current = 0.0; // start at zero current
    this.stallCurrent = stallCurrent;
    this.readingTime = readingTime;
    this.timeSinceStart = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    this.currentReadings = new LinkedList<>();
  }

  /**
   * @brief updates the intake, call every loop
   * @note currently, this only updates the current readings for the motor
   */
  public void update() {
    double amps = intakeMotor.getCurrent(CurrentUnit.AMPS); // read current
    double now = timeSinceStart.time(); // get time
    currentReadings.add(new CurrentReading(amps, now)); // add new reading

    // remove old readings
    while (!currentReadings.isEmpty() && now - currentReadings.getFirst().timeStamp > readingTime) {
      currentReadings.removeFirst();
    }

    // if there are no current readings, current is 0 (this should never happen, but just in case)
    if (currentReadings.isEmpty()) {
      current = 0.0;

    } else { // if there are current samples
      // find average of current readings:
      double sum = 0; // start counting from 0
      for (CurrentReading s : currentReadings) { // for each current reading
        sum += s.current; // add it to the sum
      }
      current = sum / currentReadings.size(); // divide sum by number of readings
    }
  }

  /**
   * @brief stops the intake from spinning
   */
  public void stop() {
    intakeMotor.setPower(0);
    intaking = false;
    ejecting = false;
    idling = false;
  }

  /**
   * @brief spins intake up to bring balls in
   */
  public void intake() {
    intakeMotor.setPower(1);
    intaking = true;
    ejecting = false;
    idling = false;
  }

  /**
   * @brief spins intake up to half speed to hold balls in the mag
   */
  public void intakeIdle() {
    intakeMotor.setPower(0.5);
    intaking = true;
    ejecting = false;
    idling = true;
  }

  /**
   * @brief spins intake up in reverse to spit balls out
   */
  public void eject() {
    intakeMotor.setPower(-1);
    ejecting = true;
    intaking = false;
    idling = false;
  }

  /**
   * @brief spins intake up in reverse to half speed to keep balls out of the mag
   */
  public void ejectIdle() {
    intakeMotor.setPower(-0.5);
    ejecting = true;
    intaking = false;
    idling = true;
  }

  /**
   * @brief sets the time over which current reading averages are computed, in seconds
   * @param timeSeconds the time over which current reading averages are computed, in seconds
   */
  public void setAverageTimeSeconds(double timeSeconds) {
    readingTime = timeSeconds;
  }

  /**
   * @brief gets the time over which current reading averages are computed, in seconds
   * @return the time over which current reading averages are computed, in seconds
   */
  public double getAverageTimeSeconds() {
    return readingTime;
  }

  /**
   * @brief sets the average current at or over which the intake is considered stalled
   * @param stallCurrentAmps the stall current of the intake motor
   */
  public void setStallCurrentAmps(double stallCurrentAmps) {
    stallCurrent = stallCurrentAmps;
  }

  /**
   * @brief gets the average current at or over which the intake is considered stalled
   * @return the currently set stall current of the intake motor, in amps
   */
  public double getStallCurrentAmps() {
    return stallCurrent;
  }

  /**
   * @brief gets if the intake is stalled
   * @return true if the current of the intake motor is over the stall current, false otherwise
   * @note does not update motor current readings, call update() to update motor current reading
   */
  public boolean isStalled() {
    return current >= stallCurrent;
  }

  /**
   * @brief gets the average current of the intake motor
   * @return the average current drawn by the intake motor, in amps
   */
  public double getAverageCurrentAmps() {
    return current;
  }

  /**
   * @brief returns if the intake is in use (busy)
   * @return true if intake is spinning at full speed, false if intake is stopped or idling
   */
  public boolean isBusy() {
    return (intaking || ejecting) && !idling;
  }

  /**
   * @brief returns if the intake is intaking
   * @return true if intake is intaking, false if ejecting or stopped
   */
  public boolean isIntaking() {
    return intaking;
  }

  /**
   * @brief returns if the intake is ejecting
   * @return true if intake is ejecting, false if intaking or stopped
   */
  public boolean isEjecting() {
    return ejecting;
  }

  /**
   * @brief returns if the intake is idling (spinning at half speed)
   * @return true if the intake is idling, false if the intake is spinning at full speed or stopped
   */
  public boolean isIdling() {
    return idling;
  }

  /**
   * @brief returns if a ball is in the intake
   * @return true if a ball is in the intake, false if the intake is empty
   * @note this is basically just a wrapper around a variable that isn't used in any core methods
   */
  public boolean isBallIn() {
    return ballIn;
  }

  /**
   * @brief sets if a ball is in the intake
   * @param ballIn true if a ball is in the intake, false if the intake is empty
   * @brief this is basically just a wrapper around a variable that isn't used in any core methods
   */
  public void setBallIn(boolean ballIn) {
    this.ballIn = ballIn;
  }
}
