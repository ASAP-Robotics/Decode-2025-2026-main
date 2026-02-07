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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

/**
 * @brief class to contain the behavior of the robot in Auto, to avoid code duplication
 */
public class plzwork extends CommonRobot {
  // stuff (variables, etc., see TeleOpRobot) goes here; TODO: update
  boolean move = true;
  boolean move1 = false;
  boolean done = false;

  public plzwork(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
    super(hardwareMap, telemetry, allianceColor, false);

    // other "Init" setup stuff goes here
  }

  /**
   * @brief to be called once, when the opMode is initialized
   */
  public void init() {
    scoringSystem.init(true, false);
  }

  /**
   * @brief to be called repeatedly, while the opMode is in init
   */
  public void initLoop() {
    scoringSystem.initLoop();
  }

  /**
   * @brief to be called once when the "start" button is pressed
   */
  public void start() {
    scoringSystem.start(true); // start scoring systems up
  }

  /**
   * @brief to be called repeatedly, every loop
   */
  public void loop(MecanumDrive drive) {
    // other stuff goes here; TODO: fill out
    if (move) {
      Actions.runBlocking(
          drive
              .actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
              .splineToLinearHeading(
                  new Pose2d(20, 0, Math.toRadians(0)),
                  Math.PI / 4,
                  new TranslationalVelConstraint(80.0))
              .build());

      move = false;
    }

    if (move1) {
      Actions.runBlocking(
          drive
              .actionBuilder(new Pose2d(20, 0, Math.toRadians(0)))
              .splineToLinearHeading(
                  new Pose2d(50, 0, Math.toRadians(0)),
                  Math.PI / 4,
                  new TranslationalVelConstraint(80.0))
              .build());

      move1 = false;
      done = true;
    }

    // update scoring systems
    // scoringSystem.setRobotPosition(); // TODO: fill out
    scoringSystem.update(true);

    // update telemetry
    telemetry.update();

    if (!move && !move1) {
      scoringSystem.shoot();
      if (scoringSystem.getState() == ScoringSystem.State.INTAKING) {
        move1 = true;
      }
    }

    while (done)
      ;
  }

  public void loop() {}

  /**
   * @brief to be called once, when the "stop" button is pressed
   */
  public void stop() {
    scoringSystem.stop(); // stop all powered movement in scoring systems
  }
}
