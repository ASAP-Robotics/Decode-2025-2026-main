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

package org.firstinspires.ftc.teamcode.autos;

import static android.os.SystemClock.sleep;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommonRobot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
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
  protected Pose2d beginPose;

  private ParallelAction auto;

  public enum paths {
    FARSIDE,
    CLOSE15,
    CLOSE12,
    ClOSE15_2GATE,
  }

  private AutoPaths autoPaths;

  private final SearchLimelight limelight;
  protected MecanumDrive drive;

  public AutoRobot(
      HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor, paths path) {
    super(hardwareMap, telemetry, allianceColor, false);

    limelight = new SearchLimelight(hardwareMap);
    autoPaths = new AutoPaths(allianceColor);
    if (allianceColor == AllianceColor.RED) {
      flipy = -1;
    }

    beginPose = new Pose2d(0, 0, 0);
    switch (path) {
      case FARSIDE:
        beginPose = new Pose2d(63, -8.6 * flipy, Math.toRadians(-90) * flipy);
        drive = new MecanumDrive(hardwareMap, beginPose);
        auto = autoPaths.getFarSideAuto(scoringSystem, drive, telemetry);
        break;
      case CLOSE15:
        beginPose = allianceColor.getAutoStartPosition();
        drive = new MecanumDrive(hardwareMap, beginPose);
        auto = autoPaths.getCloseSide15Auto(scoringSystem, drive, telemetry);
        break;
      case CLOSE12:
        beginPose = allianceColor.getAutoStartPosition();
        drive = new MecanumDrive(hardwareMap, beginPose);
        auto = autoPaths.getCloseSide12Auto(scoringSystem, drive, telemetry);
        break;
      case ClOSE15_2GATE:
        beginPose = allianceColor.getAutoStartPosition();
        drive = new MecanumDrive(hardwareMap, beginPose);
        auto = autoPaths.getCloseSide15Auto2Gate(scoringSystem, drive, telemetry);
        break;
    }
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

    Actions.runBlocking(auto);
  }

  public void stop() {}

  @Override
  public void loop() {}
}
