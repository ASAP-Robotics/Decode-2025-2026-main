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

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

public class TuningTurret extends LinearOpMode {

  @Override
  public void runOpMode() {
    DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    DcMotorEx turretRotator = hardwareMap.get(DcMotorEx.class, "turretRotator");
    Servo rawTurretHood = hardwareMap.get(Servo.class, "turretHood");
    Axon turretHood = new Axon(rawTurretHood);
    Turret turret = new Turret(flywheel, turretRotator, turretHood);
    Limelight3A rawLimelight = hardwareMap.get(Limelight3A.class, "limelight");
    Limelight limelight = new Limelight(rawLimelight, AllianceColor.BLUE, false, 2);
    turret.idle();
    turret.disable();

    waitForStart();

    while (opModeIsActive()) {
      if (gamepad1.dpadUpWasPressed()) {
        turret.testingSpeed += 100;

      } else if (gamepad1.dpadDownWasPressed()) {
        turret.testingSpeed -= 100;

      } else if (gamepad1.dpadRightWasPressed()) {
        turret.testingAngle += 5;

      } else if (gamepad1.dpadLeftWasPressed()) {
        turret.testingAngle -= 5;
      }

      limelight.update();
      turret.update();

      telemetry.addData("Limelight locked", limelight.isTargetInFrame());
      telemetry.addData("Target size", limelight.getTargetSize());
      telemetry.addData("Angle", turret.testingAngle);
      telemetry.addData("Speed", turret.testingSpeed);
      telemetry.addData("Up to speed", turret.isReadyToShoot());
      telemetry.update();
    }
  }
}
