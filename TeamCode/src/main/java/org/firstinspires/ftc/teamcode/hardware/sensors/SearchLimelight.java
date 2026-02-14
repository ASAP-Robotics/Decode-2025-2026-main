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

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.types.BallSequence;

public class SearchLimelight {
  private final Limelight3A limelight;
  private BallSequence validSequence = BallSequence.GPP;
  private boolean detected = false;

  public SearchLimelight(HardwareMap hardwareMap) {
    this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
  }

  public void init() {
    limelight.pipelineSwitch(5);
    limelight.start();
  }

  public void update() {
    LLResult result = limelight.getLatestResult();

    if (result.isValid()) {
      for (BallSequence sequence : BallSequence.values()) {
        if (sequence.getAprilTagId() == result.getFiducialResults().get(0).getFiducialId()) {
          validSequence = sequence;
          detected = true;
        }
      }
    }
  }

  public BallSequence getSequence() {
    return validSequence;
  }

  public boolean isDetected() {
    return detected;
  }
}
