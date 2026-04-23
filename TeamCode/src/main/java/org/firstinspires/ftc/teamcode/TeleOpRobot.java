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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheelBase;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.Spindex;
import org.firstinspires.ftc.teamcode.hardware.sensors.Limelight;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.firstinspires.ftc.teamcode.utils.BallSequenceFileReader;
import org.firstinspires.ftc.teamcode.utils.PositionFileReader;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

/**
 * @brief class to contain the behavior of the robot in TeleOp, to avoid code duplication
 */
@Config
public class TeleOpRobot extends CommonRobot {
  // config vars (FTC Dashboard)
  public static double MANUAL_SHOOTING_DIST = 75; // inches
  public static double MANUAL_SHOOTING_ANGLE = 180; // degrees from straight (intake)
  public static double TRIGGER_PRESSED_THRESHOLD = 0.67;
  public static double TRIGGER_RELEASED_THRESHOLD = 0.33;
  public static double TELEMETRY_UPDATE_INTERVAL = 0.67; // seconds
  public static double PINPOINT_ERROR_TIMEOUT = 1.0; // seconds

  // config vars
  private static final double TIME_TO_ENDGAME = 100; // seconds

  protected Gamepad gamepad1;
  protected Gamepad gamepad2;
  protected MecanumWheelBase wheelBase;
  protected PinpointLocalizer pinpoint;
  protected Limelight limelight;
  protected ElapsedTime telemetryTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  protected ElapsedTime pinpointErrorTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  protected SimpleTimer endgameTimer = new SimpleTimer(TIME_TO_ENDGAME);
  protected boolean endgame = false;
  private final boolean fieldCentric;
  private boolean sortingOffsetCounterUpTriggered = false;
  private boolean sortingOffsetCounterDownTriggered = false;

  public TeleOpRobot(
      HardwareMap hardwareMap,
      Telemetry telemetry,
      AllianceColor allianceColor,
      Gamepad gamepad1,
      Gamepad gamepad2,
      boolean fieldCentric) {
    super(hardwareMap, telemetry, allianceColor, true);

    pinpoint = new PinpointLocalizer(hardwareMap, new PositionFileReader().getPosition(), true);
    limelight =
        new Limelight(
            this.hardwareMap.get(Limelight3A.class, "limelight"),
            this.allianceColor
        );

    this.gamepad1 = gamepad1;
    this.gamepad2 = gamepad2;
    this.fieldCentric = fieldCentric;

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
    while (pinpoint.getState() != GoBildaPinpointDriver.DeviceStatus.READY
        && !backup.isFinished()) {
      clearSensorCache();
      pinpoint.update();
    }

    scoringSystem.setSortingMode(Spindex.SortingMode.FAST); // start unsorted
    scoringSystem.init(false, false); // initialize scoring systems

    limelight.init();
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
    telemetryTimer.reset();
    scoringSystem.start(false); // start scoring systems up
    limelight.start();
    pinpointErrorTimer.reset();
    endgameTimer.start();
  }

  /**
   * @brief to be called repeatedly, every loop
   */
  public void loop() {
    clearSensorCache();

    boolean updateTelemetry = telemetryTimer.seconds() >= TELEMETRY_UPDATE_INTERVAL;

    // switch to sorting in endgame
    if (!endgame && endgameTimer.isFinished()) {
      endgame = true;
      scoringSystem.setSortingMode(Spindex.SortingMode.SORTED);
    }

    if (updateTelemetry) {
      telemetryTimer.reset();
    }

    limelight.update();
    limelight.getRobotPosition(scoringSystem.getTurretAngle());

    // get robot position
    PoseVelocity2d velocityPose = pinpoint.update();
    boolean faulted = pinpoint.isFaulted();
    if (!faulted) pinpointErrorTimer.reset();

    if (faulted && pinpointErrorTimer.seconds() >= PINPOINT_ERROR_TIMEOUT) {
      scoringSystem.overrideAiming(MANUAL_SHOOTING_DIST, MANUAL_SHOOTING_ANGLE);
    }

    Pose2d location = pinpoint.getPose();

    // update scoring systems
    Pose2D realRobot =
        new Pose2D(
            DistanceUnit.INCH,
            location.position.x,
            location.position.y,
            AngleUnit.RADIANS,
            location.heading.toDouble());
    Pose2D virtual =
        scoringSystem.getVirtualRobotPosition(
            realRobot,
            allianceColor.getTargetLocation(),
            velocityPose.linearVel.x,
            velocityPose.linearVel.y);
    scoringSystem.setRobotPosition(virtual);

    updateDriverControls();

    scoringSystem.update(updateTelemetry);

    if (updateTelemetry) {
      if (ScoringSystem.TELEMETRY_VERBOSITY.verbosity >= ScoringSystem.Verbosity.NORMAL.verbosity) {
        boolean connected = !pinpoint.isFaulted();
        if (ScoringSystem.TELEMETRY_VERBOSITY.verbosity >= ScoringSystem.Verbosity.DEBUG.verbosity
            || !connected) {
          telemetry.addData("🧭Pinpoint connected", connected ? "✅" : "❌");
        }
      }

      if (ScoringSystem.TELEMETRY_VERBOSITY.verbosity >= ScoringSystem.Verbosity.DEBUG.verbosity) {
        telemetry.addData("🏁Endgame", endgame);
      }

      if (ScoringSystem.TELEMETRY_VERBOSITY.verbosity
          >= ScoringSystem.Verbosity.EXCESSIVE.verbosity) {
        telemetry.addData("⌚Time to endgame", endgameTimer.remaining());
        telemetry.addData("🌎Positon (real)", realRobot);
        telemetry.addData("🌐Position (virtual)", virtual);
      }
    }

    // update wheelbase
    if (!fieldCentric) {
      wheelBase.setRotation(
          AngleUnit.DEGREES.fromRadians(location.heading.toDouble())); // for field-centric control
      wheelBase.setThrottle(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
      wheelBase.update(false, gamepad1.left_trigger >= TRIGGER_PRESSED_THRESHOLD);
    } else {
      wheelBase.setRotation(
          AngleUnit.DEGREES.fromRadians(location.heading.toDouble())); // for field-centric control
      wheelBase.setThrottle(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
      wheelBase.update(true, gamepad1.left_trigger >= TRIGGER_PRESSED_THRESHOLD);
    }

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
    if (gamepad2.left_trigger > TRIGGER_PRESSED_THRESHOLD
        && gamepad2.right_trigger > TRIGGER_PRESSED_THRESHOLD) { // SHIFT + ALT
      // reset odometry !*!*!*! use with caution !*!*!*!
      if (gamepad2.xWasPressed()) {
        Pose2D location = scoringSystem.allianceColor.getResetLocation();
        pinpoint =
            new PinpointLocalizer(
                hardwareMap,
                new Pose2d(
                    location.getX(DistanceUnit.INCH),
                    location.getY(DistanceUnit.INCH),
                    location.getHeading(AngleUnit.RADIANS)),
                false);
      }

      // turret rehome !*!*!*! use with caution !*!*!*!
      if (gamepad2.yWasPressed()) {
        scoringSystem.reSyncTurretEncoder();
      }

    } else if (gamepad2.left_trigger > TRIGGER_PRESSED_THRESHOLD) { // SHIFT
      // manual sequence setting
      if (gamepad2.dpadDownWasPressed()) {
        scoringSystem.setBallSequence(BallSequence.PGP);

      } else if (gamepad2.dpadLeftWasPressed()) {
        scoringSystem.setBallSequence(BallSequence.GPP);

      } else if (gamepad2.dpadRightWasPressed()) {
        scoringSystem.setBallSequence(BallSequence.PPG);

      } else if (gamepad2.dpadUpWasPressed()) {
        scoringSystem.setBallSequence(new BallSequenceFileReader().getSequence());
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
          scoringSystem.overrideAiming(MANUAL_SHOOTING_DIST, MANUAL_SHOOTING_ANGLE);
        }
      }

      // unjam spindexer
      if (gamepad2.xWasPressed()) {
        scoringSystem.unJamSpindexer();
      }

      // color sensor enable toggle
      if (gamepad2.yWasPressed()) {
        scoringSystem.toggleColorSensorEnabled();
      }

    } else if (gamepad2.right_trigger > TRIGGER_PRESSED_THRESHOLD) { // ALT
      // manual set empty
      if (gamepad2.aWasPressed()) {
        scoringSystem.setSpindexEmpty();
      }

      // toggle sorting mode
      if (gamepad2.bWasPressed()) {
        scoringSystem.toggleSortingMode();
      }

      // turret flap angle offset
      if (gamepad2.dpadDownWasPressed()) {
        scoringSystem.adjustHoodAngleOffset(0.5);

      } else if (gamepad2.dpadUpWasPressed()) {
        scoringSystem.adjustHoodAngleOffset(-0.5);
      }

      // turret flywheel RPM offset
      if (gamepad2.dpadLeftWasPressed()) {
        scoringSystem.adjustFlywheelRpmOffset(-10);

      } else if (gamepad2.dpadRightWasPressed()) {
        scoringSystem.adjustFlywheelRpmOffset(10);
      }

      // (alt) x and y unused

    } else { // normal
      // toggle shooting mode
      if (gamepad2.bWasPressed()) {
        scoringSystem.toggleShootingMode();
      }

      // shoot
      if (gamepad2.yWasPressed()) {
        scoringSystem.shoot();
      }

      // manual intake full / pinch point empty
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
        scoringSystem.adjustTurretAngleOffset(-5);
      } else if (gamepad2.dpadUpWasPressed()) {
        scoringSystem.adjustTurretAngleOffset(5);
      }
    }

    // shoot (driver 1)
    if (gamepad1.rightBumperWasPressed()) {
      scoringSystem.shoot();
    }

    // cancel shot
    if (gamepad2.rightBumperWasPressed() || gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD) {
      scoringSystem.cancelShot();
    }

    // eject intake, unjam spindexer
    if (gamepad2.leftBumperWasPressed() || gamepad1.leftBumperWasPressed()) {
      scoringSystem.clearIntake();
      scoringSystem.unJamSpindexer();
    }

    // home spindexer (driver 1)
    if (gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD) {
      scoringSystem.homeSpindexer();
    }

    // increment sorting offset
    if (gamepad2.right_stick_y > TRIGGER_PRESSED_THRESHOLD && !sortingOffsetCounterUpTriggered) {
      sortingOffsetCounterUpTriggered = true;
      sortingOffsetCounterDownTriggered = false;
      scoringSystem.setSortingOffset((scoringSystem.getSortingOffset() + 2) % 3);
    }

    // decrement sorting offset
    if (gamepad2.right_stick_y < -TRIGGER_PRESSED_THRESHOLD && !sortingOffsetCounterDownTriggered) {
      sortingOffsetCounterDownTriggered = true;
      sortingOffsetCounterUpTriggered = false;
      scoringSystem.setSortingOffset((scoringSystem.getSortingOffset() + 1) % 3);
    }

    // stick up and down release reset logic
    if (Math.abs(gamepad2.right_stick_y) < TRIGGER_RELEASED_THRESHOLD) {
      sortingOffsetCounterDownTriggered = false;
      sortingOffsetCounterUpTriggered = false;
    }
  }
}
