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

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONObject;
import java.io.File;

public class PositionFileReader {
  private static final Pose2d DEFAULT_POSITION = new Pose2d(0, 0, 0);
  private Pose2d position;
  private boolean defaulted;

  /** Reads the saved position from file */
  public PositionFileReader() {
    try {
      File configFile = AppUtil.getInstance().getSettingsFile("auto_end_position.json");
      String raw = ReadWriteFile.readFile(configFile);

      if (raw == null || raw.trim().isEmpty()) {
        position = DEFAULT_POSITION;
        defaulted = true;
        return;
      }

      JSONObject config = new JSONObject(raw);

      if (config.getInt("version") != 1) {
        position = DEFAULT_POSITION;
        defaulted = true;
        return;
      }

      double x = config.getDouble("x");
      double y = config.getDouble("y");
      double heading = config.getDouble("heading");

      position = new Pose2d(x, y, heading);
      defaulted = false;

    } catch (Exception e) {
      // File missing, bad JSON, or invalid enum
      position = DEFAULT_POSITION;
      defaulted = true;
    }
  }

  /**
   * Gets the position
   *
   * @return the position
   */
  public Pose2d getPosition() {
    return position;
  }

  /**
   * Gets if the position was defaulted
   *
   * @return true if the position couldn't be read and was defaulted, false otherwise
   */
  public boolean isDefaulted() {
    return defaulted;
  }
}
