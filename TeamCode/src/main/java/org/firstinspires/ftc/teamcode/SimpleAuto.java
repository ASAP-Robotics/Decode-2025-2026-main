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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actions.ObeliskSearch;
import org.firstinspires.ftc.teamcode.actions.updateTelemetry;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;
import com.acmerobotics.roadrunner.Action;

public class SimpleAuto extends CommonRobot {

  private Limelight3A limelight;
  public SimpleAuto(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
    super(hardwareMap, telemetry, allianceColor, true);
    Pose2d beginPose = allianceColor.getAutoStartPosition();
    drive = new MecanumDrive(hardwareMap, beginPose);
  }

  @Override
  public void initLoop() {

  }
  public void stop() {}

  @Override
  public void loop() {}

  protected MecanumDrive drive;
  private Action obeliskSearchAction;

  public void init() {
    SimpleTimer backup = new SimpleTimer(2);
    backup.start();
    while (drive.localizer.getState() != GoBildaPinpointDriver.DeviceStatus.READY
            && !backup.isFinished()) {
      drive.localizer.update();
    }

  }

  public void start() {
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.pipelineSwitch(5);
    limelight.start();


    // ✅ Instantiate it (constructor args depend on your ObeliskSearch class)
    obeliskSearchAction = new ObeliskSearch(limelight,telemetry);

    Actions.runBlocking(
            new ParallelAction(
                    new updateTelemetry(telemetry),
                    obeliskSearchAction
            )
    );
  }
}