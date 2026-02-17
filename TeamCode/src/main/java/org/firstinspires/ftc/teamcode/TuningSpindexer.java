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
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.hardware.motors.UnidirectionalHomableRotator;
import org.firstinspires.ftc.teamcode.hardware.sensors.ElcAbsEncoderAnalog;

@TeleOp(name = "Tuning spindexer", group = "Tuning")
@Config
public class TuningSpindexer extends LinearOpMode {
  public static int target_loop_time = 20;
  public static double angle = 0;
  public static double kD = 0.0;
  public static double kI = 0.0;
  public static double kP = 0.015;

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

    ElcAbsEncoderAnalog encoder = new ElcAbsEncoderAnalog(hardwareMap, "spindexEncoder");
    MotorEx motor = new MotorEx(hardwareMap, "spindex", Motor.GoBILDA.RPM_117);
    UnidirectionalHomableRotator spindex =
        new UnidirectionalHomableRotator(motor, encoder, 0.015, 0.0, 0.0, 1, true); // true

    waitForStart();
    spindex.start();
    intake.intake();

    while (opModeIsActive()) {
      // it shouldn't matter where this goes
      double rawLoopTime = loopTime.milliseconds();
      sleep((long) Math.max(target_loop_time - rawLoopTime, 0));
      double realLoopTime = loopTime.milliseconds();
      loopTime.reset();

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
      dashboardTelemetry.addData("Loop time", realLoopTime);
      loopTime.reset();

      dashboardTelemetry.update();
    }
  }
}
