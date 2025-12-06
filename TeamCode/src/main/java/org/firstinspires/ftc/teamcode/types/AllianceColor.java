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

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public enum AllianceColor {
  RED(
      24,
      2,
      -50,
      new Pose2d(-59, 38, 0), // starting pose
      new Pose2d(-4, 31, Math.toRadians(90)), // roadrunner shooting pose
      new Pose2D(DistanceUnit.INCH, -13, 24, AngleUnit.DEGREES, 90), // scoring system shooting pose
      new Pose2d(-6, 23, Math.toRadians(90)), // auto end position
      new Pose2D(DistanceUnit.INCH, -66, 66, AngleUnit.DEGREES, 0),
      new Pose2D(DistanceUnit.INCH, 63, 63, AngleUnit.DEGREES, 0)),
  BLUE(
      20,
      1,
      0,
      new Pose2d(-59, -38, 0),
      new Pose2d(-4, -31, Math.toRadians(-90)),
      new Pose2D(DistanceUnit.INCH, -13, -24, AngleUnit.DEGREES, -90),
      new Pose2d(-61.6, -37.5, Math.toRadians(0)),
      new Pose2D(DistanceUnit.INCH, -66, -66, AngleUnit.DEGREES, 0),
      new Pose2D(DistanceUnit.INCH, 63, -63, AngleUnit.DEGREES, 0));

  private final int aprilTagId;
  private final int limelightPipeline;
  private final double turretOffset;
  private final Pose2D targetLocation;
  private final Pose2D resetLocation;
  private final Pose2d autoStartPosition;
  private final Pose2d autoRRShootPosition;
  private final Pose2D autoSSShootPosition;
  private final Pose2d autoEndPosition;

  AllianceColor(
      int aprilTagId,
      int limelightPipeline,
      double turretOffset,
      Pose2d autoStartPosition,
      Pose2d autoRRShootPosition,
      Pose2D autoSSShootPosition,
      Pose2d autoEndPosition,
      Pose2D targetLocation,
      Pose2D resetLocation) {
    this.aprilTagId = aprilTagId;
    this.limelightPipeline = limelightPipeline;
    this.turretOffset = turretOffset;
    this.autoStartPosition = autoStartPosition;
    this.autoRRShootPosition = autoRRShootPosition;
    this.autoSSShootPosition = autoSSShootPosition;
    this.autoEndPosition = autoEndPosition;
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
  public double getTurretOffset() {
    return turretOffset;
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

  /**
   * @brief gets the position of the robot at the start of auto
   * @return the auto start position of the robot
   */
  public Pose2d getAutoStartPosition() {
    return autoStartPosition;
  }

  /**
   * @brief gets the roadRunner position that the robot shoots at in auto
   * @return the auto shoot position of the robot
   */
  public Pose2d getAutoRRShootPosition() {
    return autoRRShootPosition;
  }

  /**
   * @brief gets the position to give the scoring system while shooting in auto
   * @return the auto scoring system shoot position fo the robot
   */
  public Pose2D getAutoSSShootPosition() {
    return autoSSShootPosition;
  }

  /**
   * @brief gets the position of the robot at the end of auto (to use at the start of teleOp
   * @return the position the robot finishes auto at
   */
  public Pose2d getAutoEndPosition() {
    return autoEndPosition;
  }
}
