/*
 * Copyright 2026 ASAP Robotics (FTC Team 22029)
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

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.sensors.SearchLimelight;
import org.firstinspires.ftc.teamcode.types.BallSequence;

@TeleOp(group = "test", name = "Test Obelisk Search")
public class TestObeliskSearch extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    SearchLimelight limelight = new SearchLimelight(hardwareMap);

    waitForStart();
    limelight.init();

    while (opModeIsActive()) {
      limelight.update();

      telemetry.addData("Sequence", limelight.getSequence());
      telemetry.addData("Detected", limelight.isDetected());
      telemetry.update();
    }
  }
}
