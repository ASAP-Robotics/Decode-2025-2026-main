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

import static android.os.SystemClock.sleep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actions.AutoEndShutdowAction;
import org.firstinspires.ftc.teamcode.actions.ObeliskSearch;
import org.firstinspires.ftc.teamcode.actions.setAiming;
import org.firstinspires.ftc.teamcode.actions.setScoringPose;
import org.firstinspires.ftc.teamcode.actions.shootAction;
import org.firstinspires.ftc.teamcode.actions.updateScoring;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

/** class to contain the behavior of the robot in Auto, to avoid code duplication */
public class AutoRobot extends CommonRobot {
  // stuff (variables, etc., see TeliOpRobot) goes here;
  private int flipx = 1; // flip over the y axis
  private int flipy = 1; // flip over the x axis
  private double rotate = 0;
  private final double distance = 60.4;
  private final double angle = -53;
  protected final Pose2d beginPose;

  private final int pickupAcc = 210;
  private final int fastAcc = 240;


  private Limelight3A limelight;
  protected MecanumDrive drive;

  private ObeliskSearch ObeliskSearch;

  public AutoRobot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
    super(hardwareMap, telemetry, allianceColor, false);
    beginPose = allianceColor.getAutoStartPosition();
    drive = new MecanumDrive(hardwareMap, beginPose);
  }

  public void init() {
    flipy = 1;
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.pipelineSwitch(5);
    limelight.start();
    SimpleTimer backup = new SimpleTimer(2);
    backup.start();
    drive.localizer.recalibrate();

    sleep(2000);

    while (drive.localizer.getState() != GoBildaPinpointDriver.DeviceStatus.READY
        && !backup.isFinished()) {
      drive.localizer.update();
    }

    sleep(2000);

    scoringSystem.init(true, true);
  }

  public void initLoop() {
    scoringSystem.initLoop();
  }

  public void start() {
    scoringSystem.start(true); // start scoring systems up

    if (allianceColor == AllianceColor.RED) {
      flipy = -1;
    }

    Actions.runBlocking(
        new ParallelAction( // BIGGEST BOI
            new updateScoring(scoringSystem, this, telemetry),
            // new updateTelemetry(telemetry),
            new SequentialAction( // BIG BOI
                new SequentialAction(new setScoringPose(scoringSystem, allianceColor)), // 1
                new ObeliskSearch(limelight, telemetry), // shoot 1
                new setAiming(distance, angle, 2200, 32, flipy, scoringSystem),
                new SequentialAction( // shoot 1
                    drive
                        .actionBuilder(allianceColor.getAutoStartPosition())
                        // -----SHOOT1------\\
                        .splineToLinearHeading(
                            allianceColor.getAutoRRShootPosition(),
                            (Math.PI / -8) * flipy,
                            new TranslationalVelConstraint(250.0),
                            new ProfileAccelConstraint(-50, fastAcc))
                        .build(),
                    new shootAction(scoringSystem)),
                new SequentialAction( // pickup 1
                    new setAiming(distance, angle, 2320, 32, flipy, scoringSystem),
                    drive
                        .actionBuilder(allianceColor.getAutoRRShootPosition())

                        // pickup first
                        .splineToLinearHeading(
                                new Pose2d(14 * flipx, -27.9 * flipy, flipy * (Math.toRadians(-90))),
                                (Math.PI / -2) * flipy,
                                new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, fastAcc))
                            .splineToLinearHeading(
                                    new Pose2d(15 * flipx, -61.5 * flipy, flipy * (Math.toRadians(-90))),
                                    (Math.PI / -2) * flipy,
                                    new TranslationalVelConstraint(250.0),
                                    new ProfileAccelConstraint(-10, pickupAcc))
                        .waitSeconds(0.5)
                        // shoot 2
                            .strafeToLinearHeading(allianceColor.getAutoRRShootPosition().position,flipy * (Math.toRadians(-90)),new TranslationalVelConstraint(250.0),
                                    new ProfileAccelConstraint(-50, fastAcc))
                        .build(),
                    new shootAction(scoringSystem)),
                new SequentialAction( // pickup2
                    drive
                        .actionBuilder(allianceColor.getAutoRRShootPosition())
                        .splineToLinearHeading(
                            // go to pickup2
                            new Pose2d(13 * flipx, -35 * flipy, flipy * (Math.toRadians(-120))),
                            (Math.PI / -2) * flipy,
                            new TranslationalVelConstraint(250.0),
                            new ProfileAccelConstraint(-50, fastAcc))
                            .strafeTo(new Pose2d(13 * flipx, -62 * flipy, flipy * (Math.toRadians(-118))).position)
                        .waitSeconds(3)
                        // shoot3
                            .strafeToLinearHeading(allianceColor.getAutoRRShootPosition().position,flipy * (Math.toRadians(-90)),new TranslationalVelConstraint(250.0),
                                    new ProfileAccelConstraint(-50, fastAcc))
                        .build(),
                    new shootAction(scoringSystem)),
                    new SequentialAction(
                            drive
                                    .actionBuilder(allianceColor.getAutoRRShootPosition())
                                    .splineToLinearHeading(
                                            // pickup 1
                                            new Pose2d(-11.6 * flipx, -51 * flipy, flipy * (Math.toRadians(-90))),
                                            (Math.PI / -2) * flipy,
                                            new TranslationalVelConstraint(240.0),
                                            new ProfileAccelConstraint(-50, pickupAcc))
                                    .waitSeconds(.5)
                                    // shoot3
                                    .strafeToLinearHeading(allianceColor.getAutoRRShootPosition().position,flipy * (Math.toRadians(-90)),new TranslationalVelConstraint(250.0),
                                            new ProfileAccelConstraint(-50, fastAcc))
                                    .build(),
                            new shootAction(scoringSystem)),
                new SequentialAction( // pickup 3
                    drive
                        .actionBuilder(allianceColor.getAutoRRShootPosition())
                        // go to pickup3
                        .splineToLinearHeading(
                            new Pose2d(37.1 * flipx, -25 * flipy, flipy * (Math.toRadians(-90))),
                            (Math.PI / -2) * flipy,
                            new TranslationalVelConstraint(250.0),
                            new ProfileAccelConstraint(-50, fastAcc))
                        // pickup3
                        .strafeTo(
                            new Pose2d(37.1 * flipx, -62.5 * flipy, flipy * (Math.toRadians(-90))).position)
                        .waitSeconds(0.5)
                        // shoot
                            .splineTo(
                                    allianceColor.getAutoRRShootPosition().position,
                                    (Math.PI / -2) * flipy,
                                    new TranslationalVelConstraint(250.0),
                                    new ProfileAccelConstraint(-50, fastAcc))
                        .build(),
                    new shootAction(scoringSystem)),
                new ParallelAction( // leave
                    drive
                        .actionBuilder(allianceColor.getAutoRRShootPosition())
                        // leave
                        .splineToLinearHeading(
                            new Pose2d(4.2 * flipx, -43.8 * flipy, flipy * (Math.toRadians(-90))),
                            (Math.PI / -2) * flipy,
                            new TranslationalVelConstraint(250.0),
                            new ProfileAccelConstraint(-50, fastAcc))
                        .build(),
                    new AutoEndShutdowAction(scoringSystem)))));
  }

  public void stop() {}

  @Override
  public void loop() {}
}