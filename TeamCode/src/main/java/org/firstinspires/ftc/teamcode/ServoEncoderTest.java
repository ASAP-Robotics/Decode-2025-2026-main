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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Servo encoder test", group = "Testing")
public class ServoEncoderTest extends LinearOpMode {
  private AnalogInput servoEncoder;
  public void runOpMode() {
    servoEncoder = hardwareMap.get(AnalogInput.class, "testEncoder");

    waitForStart();

    while(opModeIsActive()) {
      double voltage = servoEncoder.getVoltage();
      telemetry.addData("Servo encoder voltage", voltage);
      telemetry.update();
    }
  }
}
