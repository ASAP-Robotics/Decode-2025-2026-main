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

package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MathUtils {
  /**
   * @brief maps a number from one range to another
   * @param x the number to map
   * @param inMin the minimum of the input range
   * @param inMax the maximum of the input range
   * @param outMin the minimum of the output range
   * @param outMax the maximum of the output range
   * @return the mapped number
   */
  public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
  }

  /**
   * @brief clamps a given value between a min and max
   * @param val the value to clamp
   * @param min the minimum value
   * @param max the maximum value
   * @return the clamped value
   */
  public static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  /**
   * @brief finds the closest value to `input` that is equal to `desired` modulo `offset`
   * @param input the initial, unadjusted input value
   * @param desired the value we want to be close to
   * @param offset the size of offset steps
   * @return the optimized value
   */
  public static double closestWithOffset(double input, double desired, double offset) {
    double numOffsets = Math.round((desired - input) / offset);
    return input + numOffsets * offset;
  }

  /**
   * Normalizes an angle around a center point
   *
   * @param angle the angle to normalize
   * @param center the angle to normalize around
   * @return the normalized angle
   * @note units in degrees
   */
  public static double normalizeAround(double angle, double center) {
    return AngleUnit.normalizeDegrees(angle - center) + center;
  }

  /**
   * Gets the difference between two positions
   *
   * @param pose1 the first position
   * @param pose2 the second position
   * @return the difference between the first and second positions
   */
  public static Pose2D poseDifference(Pose2D pose1, Pose2D pose2) {
    return new Pose2D(
        DistanceUnit.INCH,
        pose1.getX(DistanceUnit.INCH) - pose2.getX(DistanceUnit.INCH),
        pose1.getY(DistanceUnit.INCH) - pose2.getY(DistanceUnit.INCH),
        AngleUnit.DEGREES,
        AngleUnit.normalizeDegrees(
            pose1.getHeading(AngleUnit.DEGREES) - pose2.getHeading(AngleUnit.DEGREES)));
  }
}
