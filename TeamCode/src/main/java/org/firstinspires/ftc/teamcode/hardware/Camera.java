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

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.CameraLowLevel;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;

/**
 * @brief placeholder class for a camera TODO: actually fill out
 */
public class Camera extends CameraLowLevel {
  private AllianceColor allianceColor;
  private boolean navigationOnly = false;

  public Camera(
      HardwareMap hardwareMap,
      String webcamName,
      YawPitchRollAngles cameraOrientation,
      AllianceColor allianceColor) {
    super(
        hardwareMap,
        false,
        webcamName,
        new Position(DistanceUnit.INCH, 0, 0, 0, 0),
        cameraOrientation);

    super.setAllowedIds(
        Set.of(
            allianceColor.getAprilTagId(),
            BallSequence.GPP.getAprilTagId(),
            BallSequence.PGP.getAprilTagId(),
            BallSequence.PPG.getAprilTagId()));

    this.allianceColor = allianceColor;
  }

  /**
   * @brief the update() method, but sets if only the navigation apriltag should be tracked
   * @param navigationOnly if true, only track the navigation apriltag for the alliance
   */
  public void update(boolean navigationOnly) {
    this.navigationOnly = navigationOnly;
    super.setSelectedId(navigationOnly ? allianceColor.getAprilTagId() : null);
    super.update();
  }

  /**
   * @brief return the ball sequence indicated by the obelisk apriltag in view
   * @return the ball sequence detected, or null if none is detected
   */
  public BallSequence getBallSequence() {
    int id = super.getLastSeenId();
    BallSequence toReturn = null;

    for (BallSequence sequence : BallSequence.values()) {
      if (sequence.getAprilTagId() == id) {
        toReturn = sequence;
        break;
      }
    }

    return toReturn;
  }

  /**
   * @brief returns the estimated distance to the navigation apriltag for the alliance
   * @return the distance to the navigation apriltag in inches, or 0 if out of frame or multiple
   *     tags tracked
   * @note the distance cannot be found if obelisk apriltags are visible and being looked for; this
   *     is a limitation of the underlying CameraLowLevel class
   */
  public double getNavigationAprilTagDistance() {
    boolean inFrame = isNavigationAprilTagInFrame();
    if (!inFrame || !navigationOnly) return 0.0;

    double x = super.getX();
    double y = super.getY();

    return Math.hypot(x, y);
  }

  /**
   * @brief returns the estimated number of degrees the camera would need to be rotated by to be
   *     pointing directly at the navigation apriltag
   * @return the number of degrees to rotate the camera by in the x axis
   */
  public double getNavigationAprilTagAngleX() {
    double x = super.getX();
    double y = super.getY();
    return Math.toDegrees(Math.atan(x / y)); // might need to invert
  }

  /**
   * @brief returns if the navigation apriltag for the alliance is in frame
   * @return true if the apriltag is in frame, false otherwise
   */
  public boolean isNavigationAprilTagInFrame() {
    return isAprilTagInFrame(allianceColor.getAprilTagId());
  }

  /**
   * @brief returns if a given apriltag is in the camera's view
   * @param aprilTagId the ID of the apriltag to look for
   * @return true if the apriltag is in frame, false otherwise
   */
  private boolean isAprilTagInFrame(int aprilTagId) {
    List<Integer> visibleTags = super.getLastVisibleIds();
    boolean toReturn = false;

    for (int id : visibleTags) {
      if (id == aprilTagId) {
        toReturn = true;
        break;
      }
    }

    return toReturn;
  }
}
