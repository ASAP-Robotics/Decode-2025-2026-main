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
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.Spindex;

/**
 * Action to set the shooting mode of the robot (fast / slow)
 */
public class SetShootingMode implements Action {
  private final ScoringSystem scoringSystem;
  private final Spindex.ShootingMode shootingMode;

  public SetShootingMode(ScoringSystem scoringSystem, Spindex.ShootingMode shootingMode) {
    this.scoringSystem = scoringSystem;
    this.shootingMode = shootingMode;
  }

  @Override
  public boolean run(@NonNull TelemetryPacket telemetryPacket) {
    scoringSystem.setShootingMode(shootingMode);
    return false;
  }
}
