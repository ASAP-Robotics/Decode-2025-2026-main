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

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Field Centric TeleOp", group = "Drive")
public class MainOpMode extends LinearOpMode {
  /*
  private final long MAG_SETTLE_MS = 200; // tune
  private ElapsedTime magTimer = new ElapsedTime();
  private boolean waitMagThenFeed = false; // are we waiting to feed after rotating?
  private int pendingShootIdx = -1; // which slot will be at the shooter when we feed
  private String colorWanted; // will be set each loop from sequence
  private String[] slots = new String[3];
  private int currentSlot = 0;
  private boolean rejectAfterFull = false; // auto-reject window active?
  private int prevFilled = 0;
  private com.qualcomm.robotcore.util.ElapsedTime feederTimer =
      new com.qualcomm.robotcore.util.ElapsedTime();
  private com.qualcomm.robotcore.util.ElapsedTime intakeTimer =
      new com.qualcomm.robotcore.util.ElapsedTime();
  private com.qualcomm.robotcore.util.ElapsedTime moveTimer =
      new com.qualcomm.robotcore.util.ElapsedTime();
  private static final long MOVE_COOLDOWN_MS = 250;
  private static final long FEED_HOLD_MS = 300;
  // --- Wanted color sequence (PGPG...) ---
  private String wantedSeq = "PG"; // only P/G allowed
  private int wantedIdx = 0;

  private String currentWanted() {
    char c = Character.toUpperCase(wantedSeq.charAt(wantedIdx));
    // Your slots store "Purple"/"Green" (capitalized), so return same for direct equals
    return (c == 'P') ? "Purple" : "Green";
  }

  private void advanceWanted() {
    wantedIdx = (wantedIdx + 1) % wantedSeq.length();
  }

  private void setWantedSequence(String code) {
    String filtered = code.toUpperCase().replaceAll("[^PG]", "");
    if (!filtered.isEmpty()) {
      wantedSeq = filtered;
      wantedIdx = 0;
    }
  }

  private boolean matchesWanted(String slotColor) {
    if (slotColor == null) return false;
    String w = currentWanted().toUpperCase();
    String s = slotColor.toUpperCase();
    // accept full word or single-letter variants
    return (w.equals("PURPLE") && (s.equals("PURPLE") || s.equals("P")))
        || (w.equals("GREEN") && (s.equals("GREEN") || s.equals("G")));
  }

  // slot i is the wanted color?
  private boolean hasWantedAt(int i) {
    String s = slots[i];
    return s != null && !"None".equalsIgnoreCase(s) && matchesWanted(s);
  }

  // any slot with wanted color (0..2), or -1
  private int wantedIndex() {
    if (hasWantedAt(0)) return 0;
    if (hasWantedAt(1)) return 1;
    if (hasWantedAt(2)) return 2;
    return -1;
  }

  // closest to shooter ((currentSlot+2)%3), or -1
  private int wantedNearIndex() {
    int shoot = (currentSlot + 2) % 3;
    int next = (shoot + 1) % 3;
    int next2 = (shoot + 2) % 3;

    if (hasWantedAt(shoot)) return shoot;
    if (hasWantedAt(next)) return next;
    if (hasWantedAt(next2)) return next2;
    return -1;
  }

  private int countBalls() {
    int n = 0;
    for (String s : slots) if (s != null) n++;
    return n;
  }

  // hardware/state
  private boolean feederActive = false;
  // servo positions (tune!)
  private static final double FEED_UP_POS = 0.20;
  private static final double FEED_DOWN_POS = 0.00;
  private final double[] positions = {0.0, 0.33, 0.66}; // adjust for servo
  private ElapsedTime runtime = new ElapsedTime();
  private double lastMoveTime = 0;
  private final double MOVE_DELAY = 400; // ms
  // private boolean hasWantedColor = false;
  private boolean waitingForClear = false;
  */
  private DcMotor frontLeft, frontRight, backLeft, backRight, flywheelMotor, intakeMotor;
  private Servo magServo, feeder;
  private IMU imu;
  private ColorSensor colorSensor;
  private DistanceSensor range;
  private Flywheel flywheel;
  private ActiveIntake intake;
  private SpindexMag mag;
  private boolean xPrev = false; // for rising-edge detect
  private boolean xToggle = false; // the thing you're toggling
  private boolean aPrev = false;

  @Override
  public void runOpMode() {
    // Initialize motors/servos/sensors
    // frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
    flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
    // frontRight = hardwareMap.get(DcMotor.class, "rightFront");
    // backLeft = hardwareMap.get(DcMotor.class, "leftBack");
    // backRight = hardwareMap.get(DcMotor.class, "rightBack");
    range = hardwareMap.get(DistanceSensor.class, "colorSensor");
    colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    feeder = hardwareMap.get(Servo.class, "feeder");
    magServo = hardwareMap.get(Servo.class, "magServo");
    intakeMotor = hardwareMap.get(DcMotor.class, "intake");

    // frontRight.setDirection(DcMotor.Direction.REVERSE);
    // backRight.setDirection(DcMotor.Direction.REVERSE);

    flywheel = new Flywheel((DcMotorEx) flywheelMotor);
    flywheel.setTargetDistance(100); // target 100 inches away
    flywheel.idle(); // set the flywheel to spin at idle speed
    flywheel.enable(); // let flywheel spin up

    intake = new ActiveIntake((DcMotorEx) intakeMotor);

    mag = new SpindexMag(intake, flywheel, magServo, feeder, colorSensor,range);

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
    SpindexMag.BallSequence wantedSequence = SpindexMag.BallSequence.PGP; // the sequence we want to shoot

    waitForStart();

    while (opModeIsActive()) {
      flywheel.update();
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
        mag.fillMag(); // fill the mag
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

    flywheel.disable(); // stop the flywheel
    intake.stop(); // stop the intake
  }
}
