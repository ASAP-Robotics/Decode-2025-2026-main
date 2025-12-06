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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;
import java.util.LinkedList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
  protected static class Result {
    public LLResult result;
    public double timestamp;

    public Result(LLResult result, double timestamp) {
      this.result = result;
      this.timestamp = timestamp;
    }
  }

  public enum LimeLightMode {
    NAVIGATION,
    IDENTIFICATION,
    UNINITIALIZED
  }

  private static final double AVERAGE_TIME = 3; // time period to average location over, seconds
  private static final double MAX_POSITION_DEVIATION = 4; // for average, inches
  private static final double MAX_ANGLE_DEVIATION = 6.7; // for average, degrees
  private static final double OUTLIER_PERCENTAGE = 0.4; // the percent of values to trim as outliers
  File configFile = AppUtil.getInstance().getSettingsFile("ball_sequence.json");
  JSONObject config = new JSONObject(); // by default, config is blank
  private final Limelight3A limelight;
  private final AllianceColor allianceColor;
  private BallSequence detectedSequence = null;
  private LimeLightMode mode;
  private final LinkedList<Result> results = new LinkedList<>();
  private boolean isResultValid = false; // if the latest result is valid (contains a target)
  private final SimpleTimer detectionTimer;
  private final ElapsedTime timeSinceStart; // timer to track time since object creation

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
    this.timeSinceStart = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  }

  /**
   * @brief initializes limelight
   * @param search if true, limelight will search for a sequence before switching to navigation
   *     mode, if false it will start navigation immediately and use the stored last detected
   *     sequence
   */
  public void init(boolean search) {
    mode = search ? LimeLightMode.IDENTIFICATION : LimeLightMode.NAVIGATION;

    if (!search) {
      try {
        config = new JSONObject(ReadWriteFile.readFile(configFile)); // get stored sequence
        detectedSequence = BallSequence.valueOf(config.getString("sequence"));
      } catch (JSONException ignored) {
        // fail silently if config read failed
        detectedSequence = BallSequence.GPP;
      }
    }

    limelight.pipelineSwitch(getPipeline());
    limelight.setPollRateHz(60);
  }

  /**
   * @brief starts limelight up
   */
  public void start() {
    limelight.start();
    limelight.pipelineSwitch(getPipeline());
    timeSinceStart.reset();
    detectionTimer.start();
  }

  /**
   * @brief gets the latest data from limelight
   * @note call every loop
   */
  public void update(Telemetry telemetry) {
    if (detectionTimer.isFinished() && mode == LimeLightMode.IDENTIFICATION) {
      detectedSequence = BallSequence.GPP; // default to GPP if search failed
      mode = LimeLightMode.NAVIGATION;
    }

    LLResult result = limelight.getLatestResult();

    isResultValid = result != null && result.isValid();
    telemetry.addData("Result valid", isResultValid);
    if (result != null) telemetry.addData("Pipeline", result.getPipelineIndex());

    telemetry.addData("LL Mode", mode);

    try {
      telemetry.addData("Num Results", results.getFirst().result.getFiducialResults().size());
    } catch (Exception e) {
      telemetry.addData("Num Results Exception", e.toString());
    }

    if (!isResultValid) return;

    double now = timeSinceStart.time();
    try {
      results.add(new Result(result, now));

      // remove old results
      while (!results.isEmpty() && now - results.getFirst().timestamp > AVERAGE_TIME) {
        results.removeFirst();
      }
    } catch (Exception ignored) {

    }

    if (!isPipelineCorrect()) limelight.pipelineSwitch(getPipeline());

    if (mode == LimeLightMode.IDENTIFICATION) {
      telemetry.addData("Update working", updateIdentification());
    }
  }

  /**
   * @brief updates stuff to do with detecting the ball sequence
   * @note only call if mode is identification
   */
  private boolean updateIdentification() {
    limelight.pipelineSwitch(0); // stupid

    List<LLResultTypes.FiducialResult> apriltags = results.getLast().result.getFiducialResults();
    int bestId = NULL;

    for (LLResultTypes.FiducialResult tag : apriltags) {
      bestId = tag.getFiducialId();
    }

    /*
    if (apriltags.size() == 2) {
      // if limelight sees two apriltags
      bestId = getBestId(apriltags);
    } else if (apriltags.size() == 1) {
      bestId = apriltags.get(0).getFiducialId();
    }
     */

    for (BallSequence sequence : BallSequence.values()) {
      // ^ for all possible ball sequences
      if (sequence.getAprilTagId() == bestId) {
        // ^ if the tag ID of the sequence matches the best tag
        detectedSequence = sequence;
        mode = LimeLightMode.NAVIGATION;

        try {
          config.put("sequence", detectedSequence.name());
          ReadWriteFile.writeFile(configFile, config.toString()); // store detected sequence
        } catch (Exception ignored) {

        }

        return true;
      }
    }

    return true;
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
    return detectedSequence == null ? BallSequence.GPP : detectedSequence;
  }

  /**
   * @brief gets the position of limelight on the field, using FTC coordinates
   * @return the 2D position of limelight on the field, or 0 if invalid
   */
  public Pose2D getPosition() {
    if (!isResultValid || results.isEmpty()) {
      return null;
    }

    // lists for raw individual values
    List<Double> xs = new LinkedList<>();
    List<Double> ys = new LinkedList<>();
    List<Double> hs = new LinkedList<>();

    // extract individual values
    for (Result res : results) {
      Pose3D pose = res.result.getBotpose();
      double x = pose.getPosition().toUnit(DistanceUnit.INCH).x;
      double y = pose.getPosition().toUnit(DistanceUnit.INCH).y;
      double h = pose.getOrientation().getYaw(AngleUnit.DEGREES);
      xs.add(x);
      ys.add(y);
      hs.add(h);
    }

    // lists for trimmed individual values
    List<Double> xt = trimOutliers(xs);
    List<Double> yt = trimOutliers(ys);
    List<Double> ht = trimOutliers(hs);

    // if spread is too large, return null
    if (isSpreadTooLarge(xt, MAX_POSITION_DEVIATION)
        || isSpreadTooLarge(yt, MAX_POSITION_DEVIATION)
        || isSpreadTooLarge(ht, MAX_ANGLE_DEVIATION)) {
      return null;
    }

    // get average values
    Double avgX = average(xt);
    Double avgY = average(yt);
    Double avgH = average(ht);

    return new Pose2D(DistanceUnit.INCH, avgX, avgY, AngleUnit.DEGREES, avgH);
  }

  /**
   * @brief trims the outliers from a list of Doubles
   * @param data the list to trim
   * @return the trimmed list
   */
  private List<Double> trimOutliers(List<Double> data) {
    if (data.size() < 3) return data; // must have enough points

    List<Double> sorted = new LinkedList<>(data);
    sorted.sort(Double::compare);

    int start = (int) (sorted.size() * Limelight.OUTLIER_PERCENTAGE);
    int end = sorted.size() - start;

    if (start >= end) return data; // this shouldn't be necessary, but just in case

    return sorted.subList(start, end);
  }

  /**
   * @param data a list of doubles
   * @return the average of the list
   * @note if for some reason the list can't be averaged, 0 will be returned
   */
  private Double average(List<Double> data) {
    try {
      double sum = 0;
      int count = 0;
      for (int i = 0; i < data.size(); i++) {
        sum += data.get(i);
        count++;
      }

      return count == 0 ? 0 : sum / count;
    } catch (Exception ignored) {
      return 0.0;
    }
  }

  /**
   * @brief checks if the spread or difference between max an min values in a list is too large
   * @param data a list of doubles to check
   * @param maxSpread the maximum allowed spread
   * @return true if the spread is above the given threshold, false otherwise
   */
  private boolean isSpreadTooLarge(List<Double> data, double maxSpread) {
    try {
      if (data.isEmpty()) return true;
    } catch (Exception ignored) {
      return true;
    }

    double min = Double.MAX_VALUE;
    double max = -Double.MAX_VALUE;
    for (double v : data) {
      if (v < min) min = v;
      if (v > max) max = v;
    }
    return (max - min) > maxSpread;
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
    return mode == LimeLightMode.NAVIGATION;
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
    try {
      return results.getLast().result.getPipelineIndex() == getPipeline();
    } catch (Exception e) {
      return false;
    }
  }
}
