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

package org.firstinspires.ftc.teamcode.types;

public enum BallSequence {
  GPP(21, BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE),
  PGP(22, BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE),
  PPG(23, BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN);

  private final BallColor[] ballColors;
  private final int aprilTagId;

  BallSequence(int aprilTagId, BallColor... ballColors) {
    this.aprilTagId = aprilTagId;
    this.ballColors = ballColors;
  }

  /**
   * Gets an array of `BallColor`s representing the sequence
   * @return an array of `BallColor`s in the sequence
   */
  public BallColor[] getBallColors() {
    return getBallColors(0);
  }

  /**
   * Gets an array of `BallColor`s representing the sequence
   * @param offset amount to offset the colors by, equivalent to the number of balls in the ramp
   * @return an array of `BallColor`s in the sequence
   */
  public BallColor[] getBallColors(int offset) {
    // this was written by ChatGPT, needs testing
    int n = ballColors.length;
    offset = ((offset % n) + n) % n;

    BallColor[] result = new BallColor[n];

    for (int i = 0; i < n; i++) {
      result[(i + offset) % n] = ballColors[i];
    }

    return result;
  }

  public int getAprilTagId() {
    return aprilTagId;
  }
}
