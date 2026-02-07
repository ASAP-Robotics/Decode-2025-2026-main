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

package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;
import org.json.JSONObject;

public class ObeliskSearch implements Action {

  private final Limelight3A limelight;
  private boolean wrote = false;

  private final File configFile = AppUtil.getInstance().getSettingsFile("ball_sequence.json");

  private final Telemetry telemetry;
  SimpleTimer maxtime = new SimpleTimer(1);

  public ObeliskSearch(Limelight3A limelight, Telemetry telemetry) {
    this.limelight = limelight;
    this.telemetry = telemetry;
    maxtime.start();
  }

  @Override
  public boolean run(@NonNull TelemetryPacket packet) {
    if (wrote) return false; // done

    LLResult result = limelight.getLatestResult();

    // Be conservative: require a valid result
    if (result == null || !result.isValid()) {
      packet.put("ll", result == null ? "no result yet" : "result invalid");
      return true; // keep waiting
    }

    // Find biggest targetArea among 21/22/23
    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

    int bestId = -1;
    double bestArea = 0.0;

    for (LLResultTypes.FiducialResult tag : tags) {
      int id = tag.getFiducialId();
      if (id == 21 || id == 22 || id == 23) {
        double area = tag.getTargetArea();
        if (area > bestArea) {
          bestArea = area;
          bestId = id;
        }
      }
    }

    packet.put("bestId", bestId);
    packet.put("bestArea", bestArea);

    if (bestId == -1 && !maxtime.isFinished()) {
      // no relevant tag seen yet
      return true;
    }


    // Convert tag -> BallSequence (rich enum constant)
    BallSequence detectedSequence = BallSequence.GPP;
    switch (bestId) {
      case 21:
        detectedSequence = BallSequence.GPP;
        break;
      case 22:
        detectedSequence = BallSequence.PGP;
        break;
      case 23:
        detectedSequence = BallSequence.PPG;
        break;
      default:
        return true;
    }
    telemetry.addData("Obelisk Tag", bestId == -1 ? "none" : bestId);
    telemetry.update();

    // Save EXACTLY like your reference Limelight class
    try {
      String raw = ReadWriteFile.readFile(configFile);
      JSONObject config =
              (raw != null && !raw.trim().isEmpty()) ? new JSONObject(raw) : new JSONObject();

      config.put("sequence", detectedSequence.name());
      config.put("search_failed", false);

      ReadWriteFile.writeFile(configFile, config.toString());
      wrote = true;

      packet.put("saved", true);
      packet.put("savedSequence", detectedSequence.name());
      return false; // finished!

    } catch (Exception e) {
      packet.put("saveError", e.getMessage());
      return true; // keep trying
    }
  }
}
