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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.motors.HomableRotator;
import org.firstinspires.ftc.teamcode.hardware.sensors.BreakBeam;

@TeleOp(name = "Tuning turret", group = "Tuning")
@Config
public class TuningTurret extends LinearOpMode {
  public static double angle = 0;
  public static double kD = 0.001;
  public static double kI = 0.05;
  public static double kP = 0.25;

  // public static double kF = 0;

  @Override
  public void runOpMode() {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    BreakBeam sensor = new BreakBeam(hardwareMap.get(DigitalChannel.class, "breakBeam"));
    Motor motor = new Motor(hardwareMap, "motor", Motor.GoBILDA.RPM_117);
    HomableRotator spindex = new HomableRotator(motor, sensor, 0, 0, 0, 1, false);

    waitForStart();
    spindex.start();

    while (opModeIsActive()) {
      spindex.setP(kP);
      spindex.setI(kI);
      spindex.setD(kD);
      spindex.setTargetAngle(angle);
      spindex.update();

      dashboardTelemetry.addData("At target", spindex.atTarget());

      dashboardTelemetry.update();
    }
  }
}
