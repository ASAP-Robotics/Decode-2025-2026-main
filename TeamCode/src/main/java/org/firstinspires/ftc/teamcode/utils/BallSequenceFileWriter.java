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

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.json.JSONObject;

import java.io.File;

public class BallSequenceFileWriter {
  private final File configFile;

  public BallSequenceFileWriter() {
    this.configFile = AppUtil.getInstance().getSettingsFile("ball_sequence.json");
  }

  /**
   * Writes a ball sequence to the config file
   * @param sequence the sequence to record
   * @return true if the sequence was successfully written to the file, false if file writing failed
   */
  public boolean writeSequence(BallSequence sequence) {
    if (sequence == null) return false;

    try {
      JSONObject config = new JSONObject();

      config.put("sequence", sequence.name());

      ReadWriteFile.writeFile(configFile, config.toString());
      return true;

    } catch (Exception e) {
      return false;
    }
  }
}
