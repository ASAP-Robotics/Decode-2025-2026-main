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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
  // public static double speed = 2000;
  // public static double angle = 0;
  // public static double speed = 0.1;

  public static double angle = 0;
  public static double kP = 0;
  public static double kI = 0;
  public static double kD = 0;

  @Override
  public void runOpMode() {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    Motor turretRotator = new Motor(hardwareMap, "turretRotator", Motor.GoBILDA.RPM_1150);
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
    Turret turret = new Turret(flywheel, turretRotator, turretHood, 1500);
    turret.idle();
    turret.enable();

    waitForStart();
    lifter.setPosition(7);
    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    intake.setPower(0);

    while (opModeIsActive()) {
      /*
      if (gamepad1.dpadUpWasPressed()) {
        speed += 50;

      } else if (gamepad1.dpadDownWasPressed()) {
        speed -= 50;

      } else if (gamepad1.dpadRightWasPressed()) {
        angle += 5;

      } else if (gamepad1.dpadLeftWasPressed()) {
        angle -= 5;
      }
       */

      if (lifter.isAtTarget() && lifter.getTargetPosition() == 100) {
        lifter.setPosition(7);
      }

      if (gamepad1.rightBumperWasPressed()) {
        lifter.setPosition(100);
      }

      // turret.setHorizontalAngle(0);
      // turret.tuneShooting(0, 0);
      turret.setHorizontalAngle(angle);
      turret.tuneHorizontalPID(kP, kI, kD);
      turret.update();

      // dashboardTelemetry.addData("Angle", turret.getTargetVerticalAngleDegrees());
      // dashboardTelemetry.addData("Target Speed", turret.testingSpeed);
      // dashboardTelemetry.addData("Speed", turret.flywheel.getVelocity() * 60 / 28);
      // dashboardTelemetry.addData("At target speed", turret.isReadyToShoot());
      // dashboardTelemetry.addData("Lifter at target", lifter.isAtTarget());
      // dashboardTelemetry.addData("Target size", limelight.getTargetSize());
      // dashboardTelemetry.addData("Limelight locked", limelight.isTargetInFrame());
      dashboardTelemetry.addData("At target", turret.isAtTarget());
      dashboardTelemetry.addData("Angle", turret.getHorizontalAngleDegrees());
      dashboardTelemetry.addData("Target angle", turret.getTargetHorizontalAngleDegrees());

      dashboardTelemetry.update();
    }
  }
}
