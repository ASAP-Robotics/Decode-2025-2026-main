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
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Full Power", group = "Test")
public class MotorFullPower extends LinearOpMode {

  @Override
  public void runOpMode() {
    // Map your motor (make sure the config name matches RC app)
    DcMotor testMotor = hardwareMap.get(DcMotor.class, "testMotor");

    telemetry.addLine("Motor Full Power ready!");
    telemetry.update();

    waitForStart();

    // Run motor full power until stop is pressed
    while (opModeIsActive()) {
      testMotor.setPower(1.0); // Full forward power

      telemetry.addData("Motor Power", "100%");
      telemetry.update();
    }

    // Stop motor when OpMode ends
    testMotor.setPower(0);
  }
}
