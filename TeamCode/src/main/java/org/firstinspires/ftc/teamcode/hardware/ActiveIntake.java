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
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

public class ActiveIntake {
  private final DcMotorEx intake_intakeMotor;
  private boolean idling = true;
  private boolean intaking = false;
  private boolean ejecting = false;
  private boolean ballIn = false;
  public org.firstinspires.ftc.teamcode.utils.SimpleTimer timer = new SimpleTimer(1);

  public ActiveIntake(DcMotorEx intakeMotor) {
    intake_intakeMotor = intakeMotor;
    intake_intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); // spin forwards
    intake_intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // brake if zero power
  }

  /**
   * @brief stops the intake from spinning
   */
  public void stop() {
    intake_intakeMotor.setPower(0);
    intaking = false;
    ejecting = false;
    idling = false;
  }

  /**
   * @brief spins intake up to bring balls in
   */
  public void intake() {
    intake_intakeMotor.setPower(1);
    intaking = true;
    idling = false;
  }

  /**
   * @brief spins intake up to half speed to hold balls in the mag
   */
  public void intakeIdle() {
    intake_intakeMotor.setPower(0.5);
    intaking = true;
    ejecting = false;
    idling = true;
  }

  /**
   * @brief spins intake up in reverse to spit balls out
   */
  public void eject() {
    intake_intakeMotor.setPower(-1);
    ejecting = true;
    idling = false;
  }

  /**
   * @brief spins intake up in reverse to half speed to keep balls out of the mag
   */
  public void ejectIdle() {
    intake_intakeMotor.setPower(-0.5);
    ejecting = true;
    intaking = false;
    idling = true;
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
