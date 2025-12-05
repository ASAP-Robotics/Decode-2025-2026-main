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
import com.qualcomm.robotcore.util.ElapsedTime;
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
  ElapsedTime loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
  ElapsedTime timer = new ElapsedTime();

  public AutoRobot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
    super(hardwareMap, telemetry, allianceColor);
  }

  public void init() {
    // TODO: put wait for pinpoint code here (it's in TeleOpRobot.java)

    scoringSystem.init(true, true);
  }

  public void initLoop() {
    scoringSystem.initLoop();
  }

  public void start() {
    scoringSystem.start(false, false); // start scoring systems up
    timer.reset();
    loopTime.reset();
  }

  @Override
  public void loop() {}

  public void stop() {}

  public void loop(MecanumDrive drive) throws InterruptedException {
    // update scoring systems

    telemetry.addData("Loop time", loopTime.seconds());
    loopTime.reset();
    //  sleep(6000);

    Actions.runBlocking(
        new ParallelAction( // BIGEST BOI
            new updateScoring(scoringSystem),
            new SequentialAction( // BIG BOI
                new SequentialAction( // 1
                    new setScoringPose(scoringSystem)),
                new SequentialAction(
                    drive
                        .actionBuilder(new Pose2d(-59, 38, Math.toRadians(0)))
                        .splineToLinearHeading(
                            new Pose2d(-2, 31, Math.toRadians(90)),
                            Math.PI / 4,
                            new TranslationalVelConstraint(200.0),
                            new ProfileAccelConstraint(-30, 175))
                        .build(),
                    new shootAction(scoringSystem)))));
  }
}
