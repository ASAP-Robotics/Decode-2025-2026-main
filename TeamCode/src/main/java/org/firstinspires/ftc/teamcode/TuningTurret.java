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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.hardware.servos.DualServo;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

@TeleOp(name = "Tuning turret", group = "Tuning")
@Config
public class TuningTurret extends LinearOpMode {
  public static double speed = 2000;
  public static double angle = 0;
  public static double kP = 70;
  public static double kI = 10;
  public static double kD = 20;
  public static double kF = 17;

  @Override
  public void runOpMode() {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    flywheel.setPIDFCoefficients(
        DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(70, 10, 20, 17));
    DcMotorEx turretRotator = hardwareMap.get(DcMotorEx.class, "turretRotator");
    DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
    Servo rawTurretHood = hardwareMap.get(Servo.class, "turretHood");
    Servo lifter1 = hardwareMap.get(Servo.class, "lifter1");
    lifter1.setDirection(Servo.Direction.REVERSE);
    Servo lifter2 = hardwareMap.get(Servo.class, "lifter2");
    AnalogInput lifterEncoder = hardwareMap.get(AnalogInput.class, "lifterEncoder");
    Axon turretHood = new Axon(rawTurretHood);
    Axon l1 = new Axon(lifter1);
    Axon l2 = new Axon(lifter2, lifterEncoder);
    DualServo lifter = new DualServo(l1, l2);
    Turret turret = new Turret(flywheel, turretRotator, turretHood);
    Limelight3A rawLimelight = hardwareMap.get(Limelight3A.class, "limelight");
    Limelight limelight = new Limelight(rawLimelight, AllianceColor.BLUE, 2);
    limelight.init(false);
    limelight.start();
    turret.activate();
    turret.enable();

    waitForStart();
    lifter.setPosition(7);
    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    intake.setPower(0.5);

    while (opModeIsActive()) {
      if (gamepad1.dpadUpWasPressed()) {
        speed += 100;

      } else if (gamepad1.dpadDownWasPressed()) {
        speed -= 100;

      } else if (gamepad1.dpadRightWasPressed()) {
        angle += 5;

      } else if (gamepad1.dpadLeftWasPressed()) {
        angle -= 5;
      }

      if (lifter.isAtTarget() && lifter.getTargetPosition() == 180) {
        lifter.setPosition(7);
      }

      if (gamepad1.rightBumperWasPressed()) {
        lifter.setPosition(100);
      }

      turret.testingSpeed = speed;
      turret.setVerticalAngle(angle);
      turret.flywheel.setPIDFCoefficients(
          DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

      limelight.update();
      turret.update();

      dashboardTelemetry.addData("Angle", turret.getTargetVerticalAngleDegrees());
      dashboardTelemetry.addData("Target Speed", turret.testingSpeed);
      dashboardTelemetry.addData("Speed", turret.flywheel.getVelocity() * 60 / 28);
      dashboardTelemetry.addData("At target speed", turret.isReadyToShoot());
      dashboardTelemetry.addData("Lifter at target", lifter.isAtTarget());
      dashboardTelemetry.addData("Target size", limelight.getTargetSize());
      dashboardTelemetry.addData("Limelight locked", limelight.isTargetInFrame());
      dashboardTelemetry.addData("Result valid", limelight.isResultValid);

      dashboardTelemetry.update();
    }
  }
}
