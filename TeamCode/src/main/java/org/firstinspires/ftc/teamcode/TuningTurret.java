/*
 * Copyright 2025-2026 ASAP Robotics (FTC Team 22029)
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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.hardware.sensors.ElcAbsEncoderAnalog;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;

@TeleOp(name = "Tuning turret", group = "Tuning")
@Config
public class TuningTurret extends LinearOpMode {
  public static int target_loop_time = 20;
  public static double angle = 0;
  public static double kP = 0.03;
  public static double kI = 0.03;
  public static double kD = 0.0009;
  public static boolean sync = false;

  @Override
  public void runOpMode() {
    ElapsedTime loopTime = new ElapsedTime();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Servo rawTurretHood = this.hardwareMap.get(Servo.class, "turretHood");
    Axon turretHood = new Axon(rawTurretHood);
    Motor turretRotator = new Motor(hardwareMap, "turretRotator", Motor.GoBILDA.RPM_1150);
    ElcAbsEncoderAnalog turretEncoder = new ElcAbsEncoderAnalog(hardwareMap, "turretEncoder");
    DcMotorEx flywheelMotor = this.hardwareMap.get(DcMotorEx.class, "flywheel");
    Turret turret = new Turret(flywheelMotor, turretRotator, turretEncoder, turretHood, 1500);

    turret.init(0);

    waitForStart();
    turret.start();
    turret.setTargetDistance(50);
    turret.enable();
    turret.activate();
    turret.tuneShooting(2500, 40);

    while (opModeIsActive()) {
      if (sync) {
        turret.syncEncoder();
        sync = false;
      }
      turret.tuneHorizontalPID(kP, kI, kD);
      turret.setHorizontalAngle(angle);
      turret.update();

      // it shouldn't matter where this goes
      double rawLoopTime = loopTime.milliseconds();
      sleep((long) Math.max(target_loop_time - rawLoopTime, 0));
      double realLoopTime = loopTime.milliseconds();
      loopTime.reset();

      dashboardTelemetry.addData("At target", turret.isAtTarget());
      dashboardTelemetry.addData("Angle", turret.getHorizontalAngleDegrees());
      dashboardTelemetry.addData("Target angle", turret.getTargetHorizontalAngleDegrees());
      dashboardTelemetry.addData("Loop time", realLoopTime);

      dashboardTelemetry.update();
    }
  }
}
