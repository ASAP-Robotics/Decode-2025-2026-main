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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.indicators.RGBIndicator;

@TeleOp
@Config
public class IndicatorTest extends LinearOpMode {
  public static RGBIndicator.Color color = RGBIndicator.Color.OFF;

  public void runOpMode() {
    RGBIndicator indicator1 = new RGBIndicator(hardwareMap, "indicator1");
    RGBIndicator indicator2 = new RGBIndicator(hardwareMap, "indicator2");

    waitForStart();

    while (opModeIsActive()) {
      indicator1.setColor(color);
      indicator2.setColor(color);
      indicator1.update();
      indicator2.update();
    }
  }
}
