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

package org.firstinspires.ftc.teamcode.types;

public enum AllianceColor {
  // TODO: tune obelisk and target angles
  RED(24, 1, -30, 0, 90),
  BLUE(20, 2, 30, -90, 0);

  private final int aprilTagId;
  private final int limelightPipeline;
  private final double obeliskAngle;
  private final double targetAngleMin;
  private final double targetAngleMax;

  AllianceColor(
      int aprilTagId,
      int limelightPipeline,
      double obeliskAngle,
      double targetAngleMin,
      double targetAngleMax) {
    this.aprilTagId = aprilTagId;
    this.limelightPipeline = limelightPipeline;
    this.obeliskAngle = obeliskAngle;
    this.targetAngleMin = targetAngleMin;
    this.targetAngleMax = targetAngleMax;
  }

  /**
   * @brief gets the navigation (goal) apriltag ID associated with the alliance color
   * @return the ID of the apriltag on this alliance color's goal
   */
  public int getAprilTagId() {
    return aprilTagId;
  }

  /**
   * @brief gets the the limelight pipeline used for tracking the apriltag on this alliance's goal
   * @return the ID of the limelight pipeline used for aiming at this alliance color's goal
   */
  public int getLimelightPipeline() {
    return limelightPipeline;
  }

  /**
   * @brief gets the angle to move the turret to to look at the obelisk
   * @return the turret angle to look at the obelisk
   */
  public double getObeliskAngle() {
    return obeliskAngle;
  }

  /**
   * @brief gets the lower end of the range of angles to look for target in
   * @return the smallest angle the target could be visible at
   */
  public double getTargetAngleMin() {
    return targetAngleMin;
  }

  /**
   * @brief gets the upper end of the range of angles to look for target in
   * @return the largest angle the target could be visible at
   */
  public double getTargetAngleMax() {
    return targetAngleMax;
  }
}
