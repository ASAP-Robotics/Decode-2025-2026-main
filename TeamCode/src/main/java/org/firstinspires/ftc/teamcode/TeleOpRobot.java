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

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheelBase;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

/**
 * @brief class to contain the behavior of the robot in TeliOp, to avoid code duplication
 */
public class TeleOpRobot extends CommonRobot {
  protected Gamepad gamepad1;
  protected Gamepad gamepad2;
  protected MecanumWheelBase wheelBase;
  protected PinpointLocalizer pinpoint;
  protected SimpleTimer odometryResetTimer = new SimpleTimer(6.7);
  protected SimpleTimer calibrateTimer = new SimpleTimer(3); // jank, fix
  protected boolean calibrated = false; // jank, fix

  public TeleOpRobot(
      HardwareMap hardwareMap,
      Telemetry telemetry,
      AllianceColor allianceColor,
      Gamepad gamepad1,
      Gamepad gamepad2) {
    super(hardwareMap, telemetry, allianceColor);

    pinpoint = new PinpointLocalizer(hardwareMap, 0.001968667109, new Pose2d(0, 0, 0));

    this.gamepad1 = gamepad1;
    this.gamepad2 = gamepad2;

    DcMotorEx frontLeft = this.hardwareMap.get(DcMotorEx.class, "leftFront");
    DcMotorEx frontRight = this.hardwareMap.get(DcMotorEx.class, "rightFront");
    DcMotorEx backLeft = this.hardwareMap.get(DcMotorEx.class, "leftBack");
    DcMotorEx backRight = this.hardwareMap.get(DcMotorEx.class, "rightBack");
    wheelBase = new MecanumWheelBase(frontLeft, frontRight, backLeft, backRight);
  }

  /**
   * @brief to be called once, when the opMode is initialized
   */
  public void init() {
    scoringSystem.init(false, false); // initialize scoring systems
    calibrateTimer.start();
  }

  /**
   * @brief to be called repeatedly, while the opMode is in init
   */
  public void initLoop() {
    scoringSystem.initLoop();
    if (calibrateTimer.isFinished() && !calibrated) {
      pinpoint.recalibrate();
      calibrated = true;
    }
  }

  /**
   * @brief to be called once when the opMode is started
   */
  public void start() {
    scoringSystem.start(false, false); // start scoring systems up
    odometryResetTimer.start();
  }

  /**
   * @brief to be called repeatedly, every loop
   */
  public void loop() {
    // manual sequence setting
    if (gamepad2.dpadDownWasPressed()) {
      scoringSystem.setBallSequence(BallSequence.PGP);

    } else if (gamepad2.dpadLeftWasPressed()) {
      scoringSystem.setBallSequence(BallSequence.GPP);

    } else if (gamepad2.dpadRightWasPressed()) {
      scoringSystem.setBallSequence(BallSequence.PPG);
    }

    // shoot
    if (gamepad2.right_trigger > 0.5) {
      // mag.shootMag(); // shoot all balls in the mag, in a sequence if possible
      scoringSystem.shootHalfSorted();
    }

    // intake
    if (gamepad2.left_trigger > 0.5) {
      scoringSystem.fillMag(); // fill the mag with any three balls

    } else if (gamepad2.leftBumperWasPressed()) {
      scoringSystem.clearIntake();
    }

    // miscellaneous backup manual controls
    if (gamepad2.bWasPressed()) {
      scoringSystem.overrideAiming(50, 0); // TODO: tune distance
    }

    // get robot position
    PoseVelocity2d velocityPose = pinpoint.update();
    Pose2d location = pinpoint.getPose();
    double velocity =
        Math.hypot(
            Math.abs(velocityPose.linearVel.x),
            Math.abs(velocityPose.linearVel.y));
    // ^ directionless velocity of the robot, in inches per second

    // update scoring systems
    scoringSystem.setRobotPosition(new Pose2D(DistanceUnit.INCH, location.position.x, location.position.y, AngleUnit.RADIANS, location.heading.toDouble()));
    scoringSystem.update();

    /*
    if (odometryResetTimer.isFinished() && velocity < 1.0) { // might need to tune
      Pose2D limelightPose = scoringSystem.getRobotPosition();
      if (limelightPose != null) {
        pinpoint.setPosition(limelightPose);
        odometryResetTimer.start();
      }
    }
     */

    // update wheelbase
    wheelBase.setRotation(AngleUnit.DEGREES.fromRadians(location.heading.toDouble())); // for field-centric control
    wheelBase.setThrottle(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
    wheelBase.update();

    // update telemetry
    telemetry.update();
  }

  /**
   * @brief to be called once, when the "stop" button is pressed
   */
  public void stop() {
    scoringSystem.stop(); // stop all powered movement in scoring systems
    wheelBase.stop(); // stop all powered movement in wheels
  }
}
