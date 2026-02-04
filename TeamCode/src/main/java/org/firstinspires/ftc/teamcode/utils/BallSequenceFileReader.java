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

package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.json.JSONObject;

public class BallSequenceFileReader {
  private static final BallSequence DEFAULT_SEQUENCE = BallSequence.GPP;
  private BallSequence sequence;
  private boolean defaulted;

  /** Reads the saved sequence from file */
  public BallSequenceFileReader() {
    try {
      File configFile = AppUtil.getInstance().getSettingsFile("ball_sequence.json");
      String raw = ReadWriteFile.readFile(configFile);

      if (raw == null || raw.trim().isEmpty()) {
        sequence = DEFAULT_SEQUENCE;
        defaulted = true;
        return;
      }

      JSONObject config = new JSONObject(raw);

      if (!config.has("sequence")) {
        sequence = DEFAULT_SEQUENCE;
        defaulted = true;
        return;
      }

      String seq = config.getString("sequence");

      sequence = BallSequence.valueOf(seq);

    } catch (Exception e) {
      // File missing, bad JSON, or invalid enum
      sequence = DEFAULT_SEQUENCE;
      defaulted = true;
    }
  }

  /**
   * Gets the ball sequence
   *
   * @return the ball sequence
   */
  public BallSequence getSequence() {
    return sequence;
  }

  /**
   * Gets if the sequence was defaulted
   *
   * @return true if the sequence couldn't be read and was defaulted, false otherwise
   */
  public boolean isDefaulted() {
    return defaulted;
  }
}
