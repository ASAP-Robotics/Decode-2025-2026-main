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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actions.AutoEndShutdowAction;
import org.firstinspires.ftc.teamcode.actions.setAiming;
import org.firstinspires.ftc.teamcode.actions.setScoringPose;
import org.firstinspires.ftc.teamcode.actions.shootAction;
import org.firstinspires.ftc.teamcode.actions.updateScoring;
import org.firstinspires.ftc.teamcode.hardware.sensors.SearchLimelight;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.firstinspires.ftc.teamcode.utils.BallSequenceFileWriter;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

/** class to contain the behavior of the robot in Auto, to avoid code duplication */
public class AutoRobot extends CommonRobot {
  // stuff (variables, etc., see TeleOpRobot) goes here;
  private int flipy = 1; // flip over the x axis
  private double rotate = 0;

  private final double angle = -53;
  protected final Pose2d beginPose;

  private final int pickupAcc = 210;
  private final int fastAcc = 240;

  private final double secondShootX = -5.5;
  private final double secondShootY = -17;


  private final SearchLimelight limelight;
  protected MecanumDrive drive;

  public AutoRobot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
    super(hardwareMap, telemetry, allianceColor, false);
    beginPose = allianceColor.getAutoStartPosition();
    limelight = new SearchLimelight(hardwareMap);
    drive = new MecanumDrive(hardwareMap, beginPose);
  }

  public void init() {
    limelight.init();
    flipy = 1;
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
    limelight.update();
  }

  public void start() {
    BallSequence sequence = limelight.getSequence(); // get ball sequence
    new BallSequenceFileWriter().writeSequence(sequence); // save sequence to file
    scoringSystem.start(true); // start scoring systems up
    scoringSystem.setBallSequence(sequence); // set ball sequence

    if (allianceColor == AllianceColor.RED) {
      flipy = -1;
    }

    Actions.runBlocking(
        new ParallelAction( // BIGGEST BOI
            new updateScoring(scoringSystem, this, telemetry),
            // new updateTelemetry(telemetry),
            new SequentialAction( // BIG BOI
                new SequentialAction(new setScoringPose(scoringSystem, allianceColor)), // 1
                //new ObeliskSearch(limelight, telemetry),
                new setAiming(30, -67, flipy, scoringSystem),
                new shootAction(scoringSystem),
                new setAiming(90, -49, flipy, scoringSystem),
                  new SequentialAction( // shoot 1
                    drive
                        .actionBuilder(allianceColor.getAutoStartPosition())
                        // -----SHOOT1------\\
                        .splineToLinearHeading(
                            allianceColor.getAutoRRShootPosition(),
                            (Math.PI / -8) * flipy,
                            new TranslationalVelConstraint(250.0),
                            new ProfileAccelConstraint(-50, fastAcc))
                        .splineToLinearHeading(
                                    new Pose2d(14, -27.9 * flipy, flipy * (Math.toRadians(-90))),
                                    (Math.PI / -2) * flipy,
                                    new TranslationalVelConstraint(250.0),
                                    new ProfileAccelConstraint(-50, fastAcc))
                        .strafeTo(
                                    new Pose2d(15, -61.5 * flipy, flipy * (Math.toRadians(-90))).position,
                                    new TranslationalVelConstraint(250.0),
                                    new ProfileAccelConstraint(-30, pickupAcc))
                        .waitSeconds(0)
                            // shoot 2
                        .strafeToLinearHeading(new Vector2d(secondShootX,secondShootY*flipy),flipy * (Math.toRadians(-90)),new TranslationalVelConstraint(250.0),
                                    new ProfileAccelConstraint(-50, fastAcc))
                        .build(),
                          new shootAction(scoringSystem)),

                new SequentialAction( // pickup2 (gate pickup)
                        //GATE PICKUP
                    drive
                        .actionBuilder((new Pose2d(secondShootX,secondShootY*flipy,flipy * (Math.toRadians(-90)))))
                            .splineToLinearHeading(
                                new Pose2d(15, -30 * flipy, flipy * Math.toRadians(-110)),
                                flipy * Math.PI/ -1,
                                new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, fastAcc))
                            .splineToLinearHeading(
                                    new Pose2d(13.5, -62 * flipy, flipy * Math.toRadians(-110)),
                                    flipy * Math.PI/ -1,
                            new TranslationalVelConstraint(250.0),
                            new ProfileAccelConstraint(-50, fastAcc))
                            //GATE PICKUP
                            .waitSeconds(1.25)
                        // shoot3
                            .strafeToLinearHeading(new Vector2d(secondShootX,secondShootY*flipy),flipy * (Math.toRadians(-90)),new TranslationalVelConstraint(250.0),
                                    new ProfileAccelConstraint(-50, fastAcc))
                        .build(),
                    new shootAction(scoringSystem)),
                    new SequentialAction(

                            drive
                                    .actionBuilder(new Pose2d(secondShootX,secondShootY*flipy,flipy * (Math.toRadians(-90))))
                                    .splineToLinearHeading(
                                            new Pose2d(43.1, -27 * flipy, flipy * (Math.toRadians(-90))),
                                            (Math.PI / -2) * flipy,
                                            new TranslationalVelConstraint(250.0),
                                            new ProfileAccelConstraint(-50, fastAcc))
                                    // pickup 4 gpp
                                    .splineToLinearHeading(
                                            new Pose2d(40.1, -62.5 * flipy, flipy * (Math.toRadians(-90))),
                                            flipy * (Math.PI/ -2)
                                            )
                                    // shoot4
                                    .strafeToLinearHeading(new Vector2d(secondShootX,secondShootY*flipy),flipy * (Math.toRadians(-90)),new TranslationalVelConstraint(250.0),
                                            new ProfileAccelConstraint(-50, fastAcc))
                                    .build(),
                            new shootAction(scoringSystem),
                    new setAiming(70, -30,flipy, scoringSystem)),
                    new SequentialAction( // pickup close
                            drive
                                    .actionBuilder((new Pose2d(secondShootX,secondShootY*flipy,flipy * (Math.toRadians(-90)))))
                                    // go to pickup 4
                                    .splineToLinearHeading(
                                            // pickup PPG first slot
                                            new Pose2d(-14.5, -49 * flipy, flipy * (Math.toRadians(-90))),
                                            (Math.PI / -2) * flipy,
                                            new TranslationalVelConstraint(180.0),
                                            new ProfileAccelConstraint(-50, pickupAcc))
                                    .waitSeconds(.8)
                                    // shoot 5 and leave3
                                    .strafeToLinearHeading(
                                            new Pose2d(secondShootX-30,secondShootY*flipy,flipy * (Math.toRadians(-90))).position,
                                            flipy * (Math.toRadians(-90)),
                                            new TranslationalVelConstraint(300.0),
                                            new ProfileAccelConstraint(-50, fastAcc+10))
                                    .build(),

                            new shootAction(scoringSystem)),
                    new AutoEndShutdowAction(scoringSystem))));
  }

  public void stop() {}

  @Override
  public void loop() {}
}