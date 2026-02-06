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

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheelBase;
import org.firstinspires.ftc.teamcode.hardware.sensors.Limelight;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

/**
 * @brief class to contain the behavior of the robot in TeleOp, to avoid code duplication
 */
public class TeleOpRobot extends CommonRobot {
  protected Gamepad gamepad1;
  protected Gamepad gamepad2;
  protected MecanumWheelBase wheelBase;
  protected PinpointLocalizer pinpoint;
  protected Limelight limelight;
  protected SimpleTimer telemetryTimer = new SimpleTimer(0.67);
  protected SimpleTimer pinpointErrorTimer = new SimpleTimer(1);
  protected SimpleTimer odometryResetTimer = new SimpleTimer(2);

  public TeleOpRobot(
      HardwareMap hardwareMap,
      Telemetry telemetry,
      AllianceColor allianceColor,
      Gamepad gamepad1,
      Gamepad gamepad2) {
    super(hardwareMap, telemetry, allianceColor, true);
    Limelight3A rawLimelight = this.hardwareMap.get(Limelight3A.class, "limelight");
    this.limelight = new Limelight(rawLimelight, this.allianceColor);

    pinpoint = new PinpointLocalizer(hardwareMap, allianceColor.getAutoEndPosition());

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
    clearSensorCache();
    SimpleTimer backup = new SimpleTimer(2);
    backup.start();
    //pinpoint.recalibrate();
    while (pinpoint.getState() != GoBildaPinpointDriver.DeviceStatus.READY
        && !backup.isFinished()) {
      clearSensorCache();
      pinpoint.update();
    }
    limelight.init();
    scoringSystem.init(false, false); // initialize scoring systems
  }

  /**
   * @brief to be called repeatedly, while the opMode is in init
   */
  public void initLoop() {
    clearSensorCache();
    scoringSystem.initLoop();
  }

  /**
   * @brief to be called once when the opMode is started
   */
  public void start() {
    clearSensorCache();
    telemetryTimer.start();
    limelight.start();
    scoringSystem.start(false, false); // start scoring systems up
    pinpointErrorTimer.start(); // maybe change
    odometryResetTimer.start();
  }

  /**
   * @brief to be called repeatedly, every loop
   */
  public void loop() {
    clearSensorCache();

    boolean updateTelemetry = telemetryTimer.isFinished();
    if (updateTelemetry) {
      telemetryTimer.start();
    }

    // get robot position
    PoseVelocity2d velocityPose = pinpoint.update();
    boolean faulted = pinpoint.isFaulted();
    if (!faulted) pinpointErrorTimer.start();

    if (faulted && pinpointErrorTimer.isFinished()) {
      scoringSystem.overrideAiming(75, 180); // TODO: tune distance
    }

    Pose2d location = pinpoint.getPose();
    double velocity =
        Math.hypot(Math.abs(velocityPose.linearVel.x), Math.abs(velocityPose.linearVel.y));
    double angleVel = Math.abs(velocityPose.angVel);
    // ^ directionless velocity of the robot, in inches per second

    // update scoring systems
    scoringSystem.setRobotPosition(
        new Pose2D(
            DistanceUnit.INCH,
            location.position.x,
            location.position.y,
            AngleUnit.RADIANS,
            location.heading.toDouble()));

    updateDriverControls();

    scoringSystem.update(updateTelemetry);

    if (updateTelemetry) telemetry.addData("Pinpoint disconnected", pinpoint.isFaulted());

    if (odometryResetTimer.isFinished() && velocity < 2 && angleVel < 0.25) {
      Pose2D limelightPose = limelight.getRobotPosition(scoringSystem.getTurretAngle());
      if (limelightPose != null) {
        pinpoint.setPose(
            new Pose2d(
                limelightPose.getX(DistanceUnit.INCH),
                limelightPose.getY(DistanceUnit.INCH),
                limelightPose.getHeading(AngleUnit.RADIANS)));
        odometryResetTimer.start();
      }
    }

    // update wheelbase
    wheelBase.setRotation(
        AngleUnit.DEGREES.fromRadians(location.heading.toDouble())); // for field-centric control
    wheelBase.setThrottle(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
    wheelBase.update();

    // update telemetry
    if (updateTelemetry) telemetry.update();
  }

  /**
   * @brief to be called once, when the "stop" button is pressed
   */
  public void stop() {
    scoringSystem.stop(); // stop all powered movement in scoring systems
    wheelBase.stop(); // stop all powered movement in wheels
  }

  /** Handles backup driver inputs */
  private void updateDriverControls() {
    if (gamepad2.left_trigger > 0.67) { // hyper shift
      // manual sequence setting
      if (gamepad2.dpadDownWasPressed()) {
        scoringSystem.setBallSequence(BallSequence.PGP);

      } else if (gamepad2.dpadLeftWasPressed()) {
        scoringSystem.setBallSequence(BallSequence.GPP);

      } else if (gamepad2.dpadRightWasPressed()) {
        scoringSystem.setBallSequence(BallSequence.PPG);
      }

      // turret rehome !*!*!*! use with caution !*!*!*!
      if (gamepad2.yWasPressed()) {
        scoringSystem.reSyncTurretEncoder();
      }

      // home spindexer
      if (gamepad2.aWasPressed()) {
        scoringSystem.homeSpindexer();
      }

      // override aiming
      if (gamepad2.bWasPressed()) {
        if (scoringSystem.isAimOverride()) {
          scoringSystem.autoAim();

        } else {
          scoringSystem.overrideAiming(75, 180); // TODO: tune distance
        }
      }

      // reset odometry !*!*!*! use with caution !*!*!*!
      if (gamepad2.xWasPressed()) {
        Pose2D location = scoringSystem.allianceColor.getResetLocation();
        pinpoint =
            new PinpointLocalizer(
                hardwareMap,
                new Pose2d(
                    location.getX(DistanceUnit.INCH),
                    location.getY(DistanceUnit.INCH),
                    location.getHeading(AngleUnit.RADIANS)));
      }

    } else { // normal
      // unjam spindexer
      if (gamepad2.bWasPressed()) {
        scoringSystem.unJamSpindexer();
      }

      // color sensor enable toggle
      if (gamepad2.yWasPressed()) {
        scoringSystem.toggleColorSensorEnabled();
      }

      // manual intake full
      if (gamepad2.xWasPressed()) {
        scoringSystem.setIntakeFull(BallColor.PURPLE);
      } else if (gamepad2.aWasPressed()) {
        scoringSystem.setIntakeFull(BallColor.GREEN);
      }

      // adjust turret offset
      if (gamepad2.dpadLeftWasPressed()) {
        scoringSystem.adjustTurretAngleOffset(1);
      } else if (gamepad2.dpadRightWasPressed()) {
        scoringSystem.adjustTurretAngleOffset(-1);
      } else if (gamepad2.dpadDownWasPressed()) {
        scoringSystem.adjustTurretAngleOffset(5);
      } else if (gamepad2.dpadUpWasPressed()) {
        scoringSystem.adjustTurretAngleOffset(-5);
      }
    }

    // shoot
    if (gamepad2.right_trigger > 0.67 || gamepad2.rightBumperWasPressed()) {
      scoringSystem.shoot();
    }

    // eject intake, unjam spindexer
    if (gamepad2.leftBumperWasPressed()) {
      scoringSystem.clearIntake();
      scoringSystem.unJamSpindexer();
    }
  }
}
