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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;
import org.json.JSONException;
import org.json.JSONObject;

public class Limelight {
  public enum LimeLightMode {
    NAVIGATION,
    IDENTIFICATION,
    UNINITIALIZED
  }

  File configFile = AppUtil.getInstance().getSettingsFile("ball_sequence.json");
  JSONObject config = new JSONObject(); // by default, config is blank
  public final Limelight3A limelight;
  private final AllianceColor allianceColor;
  private BallSequence detectedSequence;
  private LimeLightMode mode;
  public LLResult result;
  public boolean isResultValid = false; // if the latest result is valid (contains a target)
  private final SimpleTimer detectionTimer;

  /**
   * @brief makes an object of the Limelight class
   * @param limelight the Limelight3A to use
   * @param allianceColor the alliance color of the robot
   * @param searchTime the maximum amount of time (seconds) to search for a ball sequence for
   * @note search is intended to be true for auto, and false for teliop
   */
  public Limelight(Limelight3A limelight, AllianceColor allianceColor, double searchTime) {
    this.limelight = limelight;
    this.allianceColor = allianceColor;
    this.mode = LimeLightMode.UNINITIALIZED;
    this.detectionTimer = new SimpleTimer(searchTime);
  }

  /**
   * @brief initializes limelight
   * @param search if true, limelight will search for a sequence before switching to navigation
   *     mode, if false it will start navigation immediately and use the stored last detected
   *     sequence
   */
  public void init(boolean search) {
    mode = search ? LimeLightMode.IDENTIFICATION : LimeLightMode.NAVIGATION;

    switch (mode) {
      case IDENTIFICATION:
        detectionTimer.start();
        break;

      case NAVIGATION:
        try {
          config = new JSONObject(ReadWriteFile.readFile(configFile)); // get stored sequence
          detectedSequence = BallSequence.valueOf(config.getString("sequence"));
        } catch (JSONException ignored) {
          // fail silently if config read failed
        }
        break;
    }

    limelight.pipelineSwitch(getPipeline());
    limelight.setPollRateHz(60);
  }

  /**
   * @brief starts limelight up
   */
  public void start() {
    limelight.start();
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
    } else {
      isResultValid = true;
    }

    if (!isPipelineCorrect()) limelight.pipelineSwitch(getPipeline());

    if (mode == LimeLightMode.IDENTIFICATION) updateIdentification();
  }

  /**
   * @brief updates stuff to do with detecting the ball sequence
   * @note only call if mode is identification
   */
  private void updateIdentification() {
    BallSequence oldSequence = detectedSequence;

    List<LLResultTypes.FiducialResult> apriltags = result.getFiducialResults();
    int bestId = NULL;
    // NOTE: limelight *should* always see two tags...
    /*
    if (apriltags.size() == 1) {
      // ^ if limelight only sees one apriltag
      bestId = apriltags.get(0).getFiducialId();

    } else */
    if (apriltags.size() == 2) {
      // if limelight sees two apriltags
      bestId = getBestId(apriltags);
    }

    for (BallSequence sequence : BallSequence.values()) {
      // ^ for all possible ball sequences
      if (sequence.getAprilTagId() == bestId) {
        // ^ if the tag ID of the sequence matches the best tag
        detectedSequence = sequence;
        break;
      }
    }

    boolean searchFailed = detectionTimer.isFinished();

    if (detectedSequence != oldSequence || searchFailed) {
      // ^ if a new ball sequence was detected
      mode = LimeLightMode.NAVIGATION;

      if (searchFailed) {
        detectedSequence = BallSequence.GPP; // default to GPP if search failed
      }

      try {
        config.put("sequence", detectedSequence.name());
        config.put("search_failed", searchFailed);
      } catch (JSONException ignored) {

      }
      ReadWriteFile.writeFile(configFile, config.toString()); // store detected sequence
    }
  }

  /**
   * @brief gets the best of a list of (two) apriltags
   * @param apriltags the list of apriltags
   * @return the ID of the tag to use
   */
  private int getBestId(List<LLResultTypes.FiducialResult> apriltags) {
    // assumes far-left is negative, far-right is positive, and center is 0
    double bestX = allianceColor == AllianceColor.RED ? 180 : -180;
    int bestId = NULL;
    for (LLResultTypes.FiducialResult tag : apriltags) {
      switch (allianceColor) {
        case RED:
          if (tag.getTargetXDegrees() < bestX) {
            // ^ if tag is further left than previous best
            bestX = tag.getTargetXDegrees();
            bestId = tag.getFiducialId();
          }
          break;

        case BLUE:
          if (tag.getTargetXDegrees() > bestX) {
            // ^ if tag is further right than previous best
            bestX = tag.getTargetXDegrees();
            bestId = tag.getFiducialId();
          }
          break;
      }
    }
    return bestId;
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
   * @brief gets the position of limelight on the field, using FTC coordinates
   * @return the 2D position of limelight on the field
   */
  public Pose2D getPosition() {
    if (!isResultValid) return null;
    Pose3D cameraPos = result.getBotpose();
    double x = cameraPos.getPosition().toUnit(DistanceUnit.INCH).x;
    double y = cameraPos.getPosition().toUnit(DistanceUnit.INCH).y;
    double heading = cameraPos.getOrientation().getYaw(AngleUnit.DEGREES);
    return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
  }

  /**
   * @brief gets the size of the apriltag in limelight's view
   * @return the fraction of the latest frame taken up by the apriltag, or 0 if the latest frame was
   *     invalid (tag not visible)
   */
  @Deprecated
  public double getTargetSize() {
    return isResultValid ? result.getTa() : 0;
  }

  /**
   * @brief gets the left-to-right offset angle of the apriltag, in degrees
   * @return the x angle of the target, or 0 if the latest frame was invalid (tag not visible)
   * @note intended to be used as the amount the turret needs to be turned to point at the target
   */
  @Deprecated
  public double getTargetOffsetAngleDegrees() {
    return isResultValid ? result.getTx() : 0; // might need to invert, TODO: check
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
   */
  public void detectSequence() {
    mode = LimeLightMode.IDENTIFICATION;
    detectionTimer.start();
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
