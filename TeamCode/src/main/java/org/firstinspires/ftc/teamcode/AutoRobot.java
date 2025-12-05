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

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actions.setScoringPose;
import org.firstinspires.ftc.teamcode.actions.shootAction;
import org.firstinspires.ftc.teamcode.actions.updateScoring;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

/**
 * @brief class to contain the behavior of the robot in Auto, to avoid code duplication
 */
public class AutoRobot extends CommonRobot {
  // stuff (variables, etc., see TeliOpRobot) goes here;
  protected final Pose2d beginPose;
  protected MecanumDrive drive;

  public AutoRobot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
    super(hardwareMap, telemetry, allianceColor);
    beginPose = allianceColor.getAutoStartPosition();
    drive = new MecanumDrive(hardwareMap, beginPose);
  }

  public void init() {
    SimpleTimer backup = new SimpleTimer(2);
    backup.start();
    while (drive.localizer.getState() != GoBildaPinpointDriver.DeviceStatus.READY
        && !backup.isFinished()) {
      drive.localizer.update();
    }
    scoringSystem.init(true, true);
  }

  public void initLoop() {
    scoringSystem.initLoop();
  }

  public void start() {
    scoringSystem.start(false, false); // start scoring systems up

    Actions.runBlocking(
        new ParallelAction( // BIGEST BOI
            new updateScoring(scoringSystem),
            new SequentialAction( // BIG BOI
                new SequentialAction( // 1
                    new setScoringPose(scoringSystem)),
                new SequentialAction( // shoot 1
                    drive
                        .actionBuilder(allianceColor.getAutoStartPosition())
                        .splineToLinearHeading(
                            allianceColor.getAutoRRShootPosition(),
                            Math.PI / 4,
                            new TranslationalVelConstraint(200.0),
                            new ProfileAccelConstraint(-30, 175))
                        .build(),
                    new shootAction(scoringSystem)),
                new SequentialAction( // pickup 1
                    drive
                        .actionBuilder(new Pose2d(-2, 31, Math.toRadians(90)))
                        .splineToLinearHeading(
                            new Pose2d(-2, 63, Math.toRadians(90)),
                            Math.PI / 4,
                            new TranslationalVelConstraint(175.0),
                            new ProfileAccelConstraint(-10, 110))
                        .build()),
                new SequentialAction( // back and rotate
                    drive
                        .actionBuilder(new Pose2d(-2, 52, Math.toRadians(90)))
                        .splineToLinearHeading(
                            new Pose2d(-2, 48, Math.toRadians(270)),
                            Math.PI / 4,
                            new TranslationalVelConstraint(200.0),
                            new ProfileAccelConstraint(-30, 175))
                        .build()),
                new SequentialAction( // hit gate
                    drive
                        .actionBuilder(new Pose2d(-2, 48, Math.toRadians(270)))
                        .splineToLinearHeading(
                            new Pose2d(-8, 60, Math.toRadians(270)),
                            Math.PI / 4,
                            new TranslationalVelConstraint(200.0),
                            new ProfileAccelConstraint(-30, 175))
                        .waitSeconds(0.5)
                        .build()),
                new SequentialAction( // shoot 2
                    drive
                        .actionBuilder(new Pose2d(-59, 38, Math.toRadians(0)))
                        .splineToLinearHeading(
                            allianceColor.getAutoRRShootPosition(),
                            Math.PI / 4,
                            new TranslationalVelConstraint(200.0),
                            new ProfileAccelConstraint(-30, 175))
                        .build(),
                    new shootAction(scoringSystem)),
                new SequentialAction( // pickup2
                    drive
                        .actionBuilder(new Pose2d(-2, 31, Math.toRadians(90)))
                        .splineToLinearHeading(
                            new Pose2d(24, 33, Math.toRadians(90)),
                            Math.PI / 4,
                            new TranslationalVelConstraint(200.0),
                            new ProfileAccelConstraint(-30, 175))
                        .build()),
                new SequentialAction( // pickup2 also
                    drive
                        .actionBuilder(new Pose2d(21, 33, Math.toRadians(90)))
                        .splineToLinearHeading(
                            new Pose2d(24, 67, Math.toRadians(90)),
                            Math.PI / 4,
                            new TranslationalVelConstraint(175.0),
                            new ProfileAccelConstraint(-10, 110))
                        .build()),
                new SequentialAction( // shoot 3
                    drive
                        .actionBuilder(new Pose2d(-59, 38, Math.toRadians(45)))
                        .splineToLinearHeading(
                            allianceColor.getAutoRRShootPosition(),
                            Math.PI / 4,
                            new TranslationalVelConstraint(200.0),
                            new ProfileAccelConstraint(-30, 175))
                        .build(),
                    new shootAction(scoringSystem)))));
  }

  public void stop() {}

  @Override
  public void loop() {}
}
