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

import static org.firstinspires.ftc.teamcode.types.Helpers.NULL;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import org.firstinspires.ftc.teamcode.types.BallSequence;

/**
 * Simple class to use a Limelight camera to look at the Obelisk and find the correct ball sequence
 */
public class SearchLimelight {
  private static final int LIMELIGHT_PIPELINE = 5;
  private static final BallSequence DEFAULT_SEQUENCE = BallSequence.GPP;
  private final Limelight3A limelight;
  private BallSequence validSequence = DEFAULT_SEQUENCE;
  private boolean detected = false;

  public SearchLimelight(HardwareMap hardwareMap) {
    this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
  }

  /** Starts the Limelight looking for the sequence */
  public void init() {
    limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
    limelight.start();
  }

  /** Checks for tags indicating the sequence */
  public void update() {
    LLResult result = limelight.getLatestResult();

    if (result != null && result.isValid()) {
      List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
      int bestTagID = NULL;
      double bestTagArea = Double.NEGATIVE_INFINITY;

      for (LLResultTypes.FiducialResult tag : tags) {
        double tagArea = tag.getTargetArea();
        if (tagArea > bestTagArea) {
          bestTagArea = tagArea;
          bestTagID = tag.getFiducialId();
        }
      }

      for (BallSequence sequence : BallSequence.values()) {
        if (sequence.getAprilTagId() == bestTagID) {
          validSequence = sequence;
          detected = true;
          break;
        }
      }
    }
  }

  /**
   * Gets the sequence detected by the Limelight, or GPP if none have been detected
   *
   * @return the detected, or default, sequence
   */
  public BallSequence getSequence() {
    return validSequence;
  }

  /**
   * Gets if the Limelight has seen tags indicating the sequence yet
   *
   * @return true if tags have been seen, false if the default is being used
   */
  public boolean isDetected() {
    return detected;
  }
}
