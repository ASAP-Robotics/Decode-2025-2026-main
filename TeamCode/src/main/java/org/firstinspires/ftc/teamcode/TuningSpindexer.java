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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.hardware.motors.UnidirectionalHomableRotator;

@TeleOp(name = "Tuning spindexer", group = "Tuning")
@Config
public class TuningSpindexer extends LinearOpMode {
  public static int sleep = 10;
  public static double angle = 0;
  public static double kD = 0.002;
  public static double kI = 0.05;
  public static double kP = 0.1;

  public static boolean home = false;
  public static UnidirectionalHomableRotator.DirectionConstraint direction =
      UnidirectionalHomableRotator.DirectionConstraint.NONE;

  // public static double kF = 0;

  @Override
  public void runOpMode() {
    ElapsedTime loopTime = new ElapsedTime();

    ActiveIntake intake = new ActiveIntake(hardwareMap.get(DcMotorEx.class, "intake"));

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    TouchSensor sensor = hardwareMap.get(TouchSensor.class, "spindexHomer");
    Motor motor = new Motor(hardwareMap, "spindex", Motor.GoBILDA.RPM_117);
    UnidirectionalHomableRotator spindex =
        new UnidirectionalHomableRotator(motor, sensor, 0.1, 0.05, 0.002, 1, true);

    waitForStart();
    spindex.start();
    intake.intake();

    while (opModeIsActive()) {
      sleep(sleep);

      if (home) {
        spindex.home();
        home = false;
      }
      spindex.setP(kP);
      spindex.setI(kI);
      spindex.setD(kD);
      spindex.setDirectionConstraint(direction);
      spindex.setAngle(angle);
      spindex.update();
      intake.update();

      dashboardTelemetry.addData("At target", spindex.atTarget());
      dashboardTelemetry.addData("Loop time", loopTime.milliseconds());
      loopTime.reset();

      dashboardTelemetry.update();
    }
  }
}
