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
// import org.firstinspires.ftc.teamcode.actions.shootAction;
import org.firstinspires.ftc.teamcode.actions.updateTelemetry;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

/** class to contain the behavior of the robot in Auto, to avoid code duplication */
public class AutoRobot extends CommonRobot {
  // stuff (variables, etc., see TeliOpRobot) goes here;
  private int flipx = 1; // flip over the y axis
  private int flipy = 1; // flip over the x axis
  private double rotate = 0;
  protected final Pose2d beginPose;
  protected MecanumDrive drive;

  public AutoRobot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
    super(hardwareMap, telemetry, allianceColor, true);
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
    // scoringSystem.init(true, true);
  }

  public void initLoop() {
    // scoringSystem.initLoop();
  }

  public void start() {
    // scoringSystem.start(false, false); // start scoring systems up
    if (allianceColor == AllianceColor.RED) {
      // flipx = -1;
      rotate = Math.PI;
      flipy = -1;
    }

    Actions.runBlocking(
        new ParallelAction( // BIGGEST BOI
            // new updateScoring(scoringSystem),
            new updateTelemetry(telemetry),
            new SequentialAction( // BIG BOI
                new SequentialAction(new setScoringPose(scoringSystem, allianceColor)), // 1
                new SequentialAction( // shoot 1
                    drive
                        .actionBuilder(allianceColor.getAutoStartPosition())
                        .splineToLinearHeading(
                            allianceColor.getAutoRRShootPosition(),
                            (Math.PI / -8),
                            new TranslationalVelConstraint(250.0),
                            new ProfileAccelConstraint(-50, 180))
                        .build() // ,
                    // new shootAction(scoringSystem)
                    ),
                new SequentialAction( // pickup 1
                    drive
                        .actionBuilder(allianceColor.getAutoRRShootPosition())
                        // pickup
                        .splineToLinearHeading(
                            new Pose2d(-11.6 * flipx, -52 * flipy, flipy * (Math.toRadians(-90))),
                            (Math.PI / -2),
                            new TranslationalVelConstraint(175.0),
                            new ProfileAccelConstraint(-10, 110))
                        .waitSeconds(0.1)
                        // goback
                        .splineToLinearHeading(
                            new Pose2d(-5.1 * flipx, -45.3 * flipy, flipy * (Math.toRadians(-90))),
                            (Math.PI / -2),
                            new TranslationalVelConstraint(250.0),
                            new ProfileAccelConstraint(-50, 180))
                        // hitgate
                        .splineToLinearHeading(
                            new Pose2d(-5.1 * flipx, -52 * flipy, flipy * (Math.toRadians(-90))),
                            (Math.PI / -2),
                            new TranslationalVelConstraint(200.0),
                            new ProfileAccelConstraint(-30, 175))
                        .waitSeconds(0.5)
                        .strafeTo(allianceColor.getAutoRRShootPosition().position)
                        .build()),

                // ,
                // new shootAction(scoringSystem)

                new SequentialAction( // pickup2
                    drive
                        .actionBuilder(allianceColor.getAutoRRShootPosition())
                        .splineToLinearHeading(
                            new Pose2d(14 * flipx, -27.9 * flipy, flipy * (Math.toRadians(-90))),
                            (Math.PI / -2),
                            new TranslationalVelConstraint(250.0),
                            new ProfileAccelConstraint(-50, 180))
                        .splineToLinearHeading(
                            new Pose2d(15 * flipx, -60.9 * flipy, flipy * (Math.toRadians(-90))),
                            (Math.PI / -2),
                            new TranslationalVelConstraint(160.0),
                            new ProfileAccelConstraint(-10, 75))
                        .waitSeconds(0.1)
                        .strafeTo(
                            (new Pose2d(15 * flipx, -49 * flipy, flipy * (Math.toRadians(-90))))
                                .position)
                        .strafeTo(allianceColor.getAutoRRShootPosition().position)
                        .build()), // ,
                // new shootAction(scoringSystem)

                new SequentialAction( // pickup 3
                    drive
                        .actionBuilder(allianceColor.getAutoRRShootPosition())
                        .splineToLinearHeading(
                            new Pose2d(37.1 * flipx, -28 * flipy, flipy * (Math.toRadians(-90))),
                            (Math.PI / -2),
                            new TranslationalVelConstraint(250.0),
                            new ProfileAccelConstraint(-50, 180))
                        .splineToLinearHeading(
                            new Pose2d(37.1 * flipx, -59.2 * flipy, flipy * (Math.toRadians(-90))),
                            (Math.PI / -2),
                            new TranslationalVelConstraint(175.0),
                            new ProfileAccelConstraint(-10, 110))
                        .waitSeconds(0.1)
                        .strafeTo(allianceColor.getAutoRRShootPosition().position)
                        .build() // ,
                    // new shootAction(scoringSystem)
                    ),
                new ParallelAction( // leave
                    drive
                        .actionBuilder(allianceColor.getAutoRRShootPosition())
                        .splineToLinearHeading(
                            new Pose2d(4.2 * flipx, -43.8 * flipy, flipy * (Math.toRadians(-90))),
                            (Math.PI / -2),
                            new TranslationalVelConstraint(250.0),
                            new ProfileAccelConstraint(-50, 180))
                        .build()))));
  }

  public void stop() {}

  @Override
  public void loop() {}
}
/*
  Actions.runBlocking(
            new ParallelAction( // BIGGEST BOI
                new updateScoring(scoringSystem),
                new updateTelemetry(telemetry),
                new SequentialAction( // BIG BOI
                    new SequentialAction(new setScoringPose(scoringSystem, allianceColor)), // 1
                    new SequentialAction( // shoot 1
                        drive
                            .actionBuilder(allianceColor.getAutoStartPosition())
                            .splineToLinearHeading(
                                new Pose2d(-7, -31, Math.toRadians(-90)),
                                Math.PI / 4,
                                new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 180))
                            .build(),
                        new shootAction(scoringSystem)),
                    new SequentialAction( // pickup 1
                        drive
                            .actionBuilder(new Pose2d(-7, -31, Math.toRadians(-90)))
                            .splineToLinearHeading(
                                new Pose2d(-2, -62, Math.toRadians(-90)),
                                Math.PI / 4,
                                new TranslationalVelConstraint(175.0),
                                new ProfileAccelConstraint(-10, 110))
                            .build()),

                    new SequentialAction( // back and rotate
                        drive
                            .actionBuilder(new Pose2d(-2, 63, Math.toRadians(90)))
                            .splineToLinearHeading(
                                new Pose2d(-8, 48, Math.toRadians(270)),
                                Math.PI / 4,
                                new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 180))
                            .build()),
                    new SequentialAction( // hit gate
                        drive
                            .actionBuilder(new Pose2d(-8, 48, Math.toRadians(270)))
                            .splineToLinearHeading(
                                new Pose2d(-8, 60, Math.toRadians(270)),
                                Math.PI / 4,
                                new TranslationalVelConstraint(200.0),
                                new ProfileAccelConstraint(-30, 175))
                            .waitSeconds(0.5)
                            .build()),

                    new SequentialAction( // shoot 2
                                          drive
                                                  .actionBuilder(new Pose2d(-2, -62, Math.toRadians(-90)))
        .splineToLinearHeading(
                                new Pose2d(-9, -32, Math.toRadians(-90)),
Math.PI / 4,
        new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 180))
        .build(),
                        new shootAction(scoringSystem)),
        new SequentialAction( // pickup2
                              drive
                                      .actionBuilder(new Pose2d(-9, -32, Math.toRadians(-90)))
        .splineToLinearHeading(
                                new Pose2d(20, -33, Math.toRadians(-90)),
Math.PI / 4,
        new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 180))
        .build()),
        new SequentialAction( // pickup2 also
                              drive
                                      .actionBuilder(new Pose2d(20, -33, Math.toRadians(-90)))
        .splineToLinearHeading(
                                new Pose2d(24, -69, Math.toRadians(-90)),
Math.PI / 4,
        new TranslationalVelConstraint(160.0),
                                new ProfileAccelConstraint(-10, 75))
        .build()),
        new SequentialAction( // shoot 3
                              drive
                                      .actionBuilder(new Pose2d(24, -69, Math.toRadians(-90)))
        .splineToLinearHeading(
                                new Pose2d(-7, -31, Math.toRadians(-90)),
Math.PI / 4,
        new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 180))
        .build(),
                        new shootAction(scoringSystem)),
        new SequentialAction( // pickup 3
                              drive
                                      .actionBuilder(new Pose2d(-2, -31, Math.toRadians(-90)))
        .splineToLinearHeading(
                                new Pose2d(38, -28, Math.toRadians(-90)),
Math.PI / 4,
        new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 180))
        .splineToLinearHeading(
                                new Pose2d(43, -69, Math.toRadians(-90)),
Math.PI / 4,
        new TranslationalVelConstraint(175.0),
                                new ProfileAccelConstraint(-10, 110))
        .build()),
        new SequentialAction( // shoot 4
                              drive
                                      .actionBuilder(new Pose2d(43, -69, Math.toRadians(-90)))
        .splineToLinearHeading(
                                new Pose2d(-7, -31, Math.toRadians(-90)),
Math.PI / 4,
        new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 180))
        .build(),
                        new shootAction(scoringSystem)),
        new ParallelAction( // leave
                            drive
                                    .actionBuilder(new Pose2d(-7, -31, Math.toRadians(-90)))
        .splineToLinearHeading(
                                new Pose2d(7, -31, Math.toRadians(-90)),
Math.PI / 4,
        new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 180))
        .build()))));
 */
