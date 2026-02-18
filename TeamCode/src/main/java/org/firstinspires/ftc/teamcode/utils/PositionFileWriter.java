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

public class PositionFileWriter {
  private final File configFile;

  public PositionFileWriter() {
    this.configFile = AppUtil.getInstance().getSettingsFile("auto_end_position.json");
  }

  /**
   * Writes a Pose2d to the config file
   * @param position the position to save
   * @return true if saved successfully, false if saving failed
   */
  public boolean writePosition(Pose2d position) {
    if (position == null) return false;

    try {
      JSONObject config = new JSONObject();

      config.put("version", 1);
      config.put("x", position.position.x);
      config.put("y", position.position.y);
      config.put("heading", position.heading.toDouble());

      ReadWriteFile.writeFile(configFile, config.toString());

      return true;

    } catch (Exception e) {
      return false;
    }
  }
}
