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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.Spindex;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.types.BallSequence;

@TeleOp(name = "Main TeliOp", group = "Drive")
public class MainOpMode extends LinearOpMode {
  // stuff was here
  private DcMotorEx frontLeft,
      frontRight,
      backLeft,
      backRight,
      flywheelMotor,
      intakeMotor,
      turretRotator;
  private Servo magServo, feeder, turretHood;
  private IMU imu;
  private ColorSensor colorSensor;
  private DistanceSensor distanceSensor;
  private Turret turret;
  private ActiveIntake intake;
  private Spindex spindex;
  private ScoringSystem mag;
  private boolean xPrev = false; // for rising-edge detect
  private boolean xToggle = false; // the thing you're toggling
  private boolean aPrev = false;

  // this is a super long line of comments and stuff to get the automatic formating to actually do
  // something. This really is quite a lot of text. Wow. Super long. Crazy. The formating had better
  // work!

  @Override
  public void runOpMode() {
    BallSequence wantedSequence = BallSequence.PGP; // the sequence we want to shoot

    // Initialize motors/servos/sensors
    // frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
    // frontRight = hardwareMap.get(DcMotor.class, "rightFront");
    // backLeft = hardwareMap.get(DcMotor.class, "leftBack");
    // backRight = hardwareMap.get(DcMotor.class, "rightBack");
    // TODO: add turret motor configuration
    turretRotator = hardwareMap.get(DcMotorEx.class, "turretRotator");
    flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
    intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
    colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    // TODO: add turret servo configuration
    turretHood = hardwareMap.get(Servo.class, "turretHood");
    feeder = hardwareMap.get(Servo.class, "feeder");
    magServo = hardwareMap.get(Servo.class, "magServo");

    // frontRight.setDirection(DcMotor.Direction.REVERSE);
    // backRight.setDirection(DcMotor.Direction.REVERSE);

    turret = new Turret(flywheelMotor, turretRotator, turretHood);
    intake = new ActiveIntake(intakeMotor);
    spindex = new Spindex(magServo, feeder, colorSensor, distanceSensor);

    mag = new ScoringSystem(intake, turret, spindex, telemetry);
    mag.setTargetDistance(100); // PLACEHOLDER

    // stuff was here
    // IMU
    // imu = hardwareMap.get(IMU.class, "imu");
    //  IMU.Parameters imuParams =
    //          new IMU.Parameters(
    //                 new RevHubOrientationOnRobot(
    //                          RevHubOrientationOnRobot.LogoFacingDirection.UP,
    //                          RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
    //  imu.initialize(imuParams);
    //  imu.resetYaw();
    // stuff was here (setting wanted sequence)

    waitForStart();

    mag.start(); // start scoring systems up

    while (opModeIsActive()) {
      mag.update();
      // --- X toggle ---
      boolean xNow = gamepad1.x;
      if (xNow && !xPrev) { // rising edge
        xToggle = !xToggle; // flip the state
      }
      xPrev = xNow;
      // stuff was here

      // shoot
      if (gamepad1.right_trigger > 0.25) {
        mag.shootSequence(wantedSequence); // shoot the desired sequence
      }

      // stuff was here

      // Button edge
      boolean aNow = gamepad1.a;
      boolean aPressed = aNow && !aPrev;
      aPrev = aNow;

      // fil mag
      if (aPressed) {
        mag.fillMagSorted(); // fill the mag
      }

      // stuff was here
      // --- Field-centric drive ---
      /* double y = -gamepad1.left_stick_y; // Forward/back
      double x = gamepad1.left_stick_x;  // Strafe
      double rx = gamepad1.right_stick_x;// Rotation

      double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

      double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
      double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

      double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
      double flPower = (rotY + rotX + rx) / denominator;
      double blPower = (rotY - rotX + rx) / denominator;
      double frPower = (rotY - rotX - rx) / denominator;
      double brPower = (rotY + rotX - rx) / denominator;

      frontLeft.setPower(flPower);
      backLeft.setPower(blPower);
      frontRight.setPower(frPower);
      backRight.setPower(brPower); */
      // telemetry was here
      telemetry.update();
    }

    mag.stop(); // stop all powered movement in scoring systems
  }
}
