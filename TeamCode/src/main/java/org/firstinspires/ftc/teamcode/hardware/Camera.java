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

import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;

/**
 * @brief placeholder class for a camera
 * TODO: actually fill out
 */
public class Camera {
  private AllianceColor allianceColor;
  public Camera(AllianceColor allianceColor) {
    this.allianceColor = allianceColor;
  }

  /**
   * @brief return the ball sequence indicated by the obelisk apriltag in view
   * @return the ball sequence detected, or null if none is detected
   */
  public BallSequence getBallSequence() {
    return null; // placeholder
  }

  /**
   * @brief returns the estimated distance to the navigation apriltag for the alliance
   * @return the distance to the navigation apriltag in inches
   */
  public double getNavigationAprilTagDistance() {
    return 50.0; // placeholder
  }

  /**
   * @brief returns the estimated number of degrees the camera would need to be rotated by to be
   * pointing directly at the navigation apriltag
   * @return the number of degrees to rotate the camera by in the x axis
   */
  public double getNavigationAprilTagAngleX() {
    return 0.0; // placeholder
  }

  /**
   * @brief returns the estimated number of degrees the camera would need to be elevated by to be
   * pointing directly at the navigation apriltag
   * @return the number of degrees to rotate the camera by in the y axis
   */
  public double getNavigationAprilTagAngleY() {
    return 0.0; // placeholder
  }

  /**
   * @brief returns if the navigation apriltag for the alliance is in frame
   * @return true if the apriltag is in frame, false otherwise
   */
  public boolean isNavigationAprilTagInFrame() {
    return isAprilTagInFrame(allianceColor.getAprilTagId()); // placeholder
  }

  /**
   * @brief returns if a given apriltag is in the camera's view
   * @param aprilTagId the ID of the apriltag to look for
   * @return true if the apriltag is in frame, false otherwise
   */
  private boolean isAprilTagInFrame(int aprilTagId) {
    return true; // placeholder
  }
}
