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

import static org.firstinspires.ftc.teamcode.types.Helpers.NULL;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;
import java.util.List;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.json.JSONException;
import org.json.JSONObject;

public class Limelight {
  public enum LimeLightMode {
    NAVIGATION,
    IDENTIFICATION
  }

  File configFile = AppUtil.getInstance().getSettingsFile("ball_sequence.json");
  JSONObject config;
  private final Limelight3A limelight;
  private final AllianceColor allianceColor;
  private BallSequence detectedSequence;
  private LimeLightMode mode;
  private LLResult result;
  private boolean isResultValid = false; // if the latest result is valid (contains a target)

  /**
   * @brief makes an object of the Limelight class
   * @param limelight the Limelight3A to use
   * @param allianceColor the alliance color of the robot
   * @param search if true, limelight will search for a sequence before switching to navigation
   *     mode, if false it will start navigation immediately and use the stored last detected
   *     sequence
   * @note search is intended to be true for auto, and false for teliop
   */
  public Limelight(Limelight3A limelight, AllianceColor allianceColor, boolean search) {
    this.limelight = limelight;
    this.allianceColor = allianceColor;
    this.mode = search ? LimeLightMode.IDENTIFICATION : LimeLightMode.NAVIGATION;
    if (search) {
      this.config = new JSONObject(); // make blank JSON object

    } else {
      try {
        this.config = new JSONObject(ReadWriteFile.readFile(configFile)); // get stored sequence
        this.detectedSequence = BallSequence.valueOf(config.getString("sequence"));
      } catch (JSONException ignored) {
        this.config = new JSONObject(); // make blank JSON object
      }
    }
    this.limelight.pipelineSwitch(getPipeline());
    this.limelight.start();
  }

  /**
   * @brief gets the latest data from limelight
   * @note call every loop
   */
  public void update() {
    result = limelight.getLatestResult();
    if (result == null || !result.isValid()) {
      isResultValid = false;
      return;
    }
    if (!isPipelineCorrect()) {
      limelight.pipelineSwitch(getPipeline());
    }

    if (mode == LimeLightMode.IDENTIFICATION) {
      BallSequence oldSequence = detectedSequence;

      List<LLResultTypes.FiducialResult> apriltags = result.getFiducialResults();
      if (apriltags.size() == 1) {
        // ^ if limelight only sees one apriltag
        for (BallSequence sequence : BallSequence.values()) {
          // ^ for each possible ball sequence
          if (sequence.getAprilTagId() == apriltags.get(0).getFiducialId()) {
            // ^ if the ID matches
            detectedSequence = sequence;
            break;
          }
        }
      }

      if (detectedSequence != oldSequence) {
        // ^ if a new ball sequence was detected
        mode = LimeLightMode.NAVIGATION;
        try {
          config.put("sequence", detectedSequence.name());
        } catch (JSONException ignored) {

        }
        ReadWriteFile.writeFile(configFile, config.toString()); // store detected sequence
      }
    }
  }

  /**
   * @brief gets the ball sequence detected by limelight or stored from last detection
   * @return the ball sequence detected by limelight
   * @note if a sequence hasn't been detected yet, this will return null
   */
  public BallSequence getSequence() {
    return detectedSequence;
  }

  /**
   * @brief gets the size of the apriltag in limelight's view
   * @return the fraction of the latest frame taken up by the apriltag, or 0 if the latest frame was
   *     invalid (tag not visible)
   */
  public double getTargetSize() {
    return isResultValid ? result.getTa() : 0;
  }

  /**
   * @brief gets the left-to-right offset angle of the apriltag, in degrees
   * @return the x angle of the target, or 0 if the latest frame was invalid (tag not visible)
   * @note intended to be used as the amount the turret needs to be turned to point at the target
   */
  public double getTargetOffsetAngleDegrees() {
    return isResultValid ? result.getTx() : 0; // might need to invert
  }

  /**
   * @brief gets if a valid target is in frame (the latest frame/result was valid)
   * @note intended for use mainly once mode is navigation
   */
  public boolean isTargetInFrame() {
    return isResultValid;
  }

  /**
   * @brief gets if limelight is ready to use for navigation
   * @return true if limelight has detected a valid ball sequence and switched to navigation mode,
   *     false otherwise
   */
  public boolean isReadyToNavigate() {
    return mode == LimeLightMode.NAVIGATION && detectedSequence != null;
  }

  /**
   * @brief gets the current mode of limelight
   * @return the mode limelight is in
   */
  public LimeLightMode getMode() {
    return mode;
  }

  /**
   * @brief forces limelight to (re) detect the ball sequence
   * @note intended only for emergency use by the driver (as a backup if something went wrong)
   */
  public void detectSequence() {
    mode = LimeLightMode.IDENTIFICATION;
  }

  /**
   * @brief gets the pipeline to use for the mode
   * @return the limelight pipeline to use
   * @note returns an invalid pipeline (-1) if mode is uninitialized
   */
  protected int getPipeline() {
    switch (mode) {
      case NAVIGATION:
        return allianceColor.getLimelightPipeline();

      case IDENTIFICATION:
        return 0;

      default:
        return NULL;
    }
  }

  /**
   * @brief gets if the pipeline used for the latest result was the correct one (if limelight has
   *     switched yet)
   * @return true if the pipeline used for the last result matches the mode, false otherwise
   */
  protected boolean isPipelineCorrect() {
    return result.getPipelineIndex() == getPipeline();
  }
}
