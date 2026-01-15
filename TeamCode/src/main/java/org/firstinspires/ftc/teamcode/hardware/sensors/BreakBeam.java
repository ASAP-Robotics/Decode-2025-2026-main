/*
 * Copyright 2026 ASAP Robotics (FTC Team 22029)
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

package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Class to use a simple break beam sensor
 * @note assumes an always-on LED
 */
public class BreakBeam {
  public enum State {
    BROKEN,
    UNBROKEN
  }

  protected static final boolean BROKEN_BEAM_STATE = false; // the state when the beam is broken
  protected final DigitalChannel sensor;

  public BreakBeam(DigitalChannel sensor) {
    this.sensor = sensor;
    this.sensor.setMode(DigitalChannel.Mode.INPUT);
  }

  /**
   * Gets the current state of the sensor's beam
   * @return the state of the beam
   */
  public State getState() {
    if (this.sensor.getState() == BROKEN_BEAM_STATE) {
      return State.BROKEN;
    } else {
      return State.UNBROKEN;
    }
  }

  /**
   * Gets if the beam is broken
   * @return true if the beam is broken, false otherwise
   */
  public boolean isBroken() {
    return this.getState() == State.BROKEN;
  }

  /**
   * Gets if the beam is unbroken
   * @return true if the beam is unbroken, false otherwise
   */
  public boolean isUnbroken() {
    return this.getState() == State.UNBROKEN;
  }
}
