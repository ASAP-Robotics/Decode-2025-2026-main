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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public enum AllianceColor {
  // TODO: tune obelisk and target angles
  RED(
      24, 2, -30, new Pose2D(DistanceUnit.INCH, -66, 66, AngleUnit.DEGREES, 0),
      new Pose2D(DistanceUnit.INCH, 63, -63, AngleUnit.DEGREES, 0)),
  BLUE(
      20, 1, 30, new Pose2D(DistanceUnit.INCH, -66, -66, AngleUnit.DEGREES, 0),
      new Pose2D(DistanceUnit.INCH, 63, 63, AngleUnit.DEGREES, 0));

  private final int aprilTagId;
  private final int limelightPipeline;
  private final double obeliskAngle;
  private final Pose2D targetLocation;
  private final Pose2D resetLocation;

  AllianceColor(
      int aprilTagId,
      int limelightPipeline,
      double obeliskAngle,
      Pose2D targetLocation,
      Pose2D resetLocation) {
    this.aprilTagId = aprilTagId;
    this.limelightPipeline = limelightPipeline;
    this.obeliskAngle = obeliskAngle;
    this.targetLocation = targetLocation;
    this.resetLocation = resetLocation;
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
   * @brief gets the location of the target for the aliance color
   * @return this alliance's target location
   */
  public Pose2D getTargetLocation() {
    return targetLocation;
  }

  /**
   * @brief gets the location of the robot when it's location is reset
   * @return this alliance's reset location
   */
  public Pose2D getResetLocation() {
    return resetLocation;
  }
}
