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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Field Centric TeleOp", group = "Drive")
public class FieldCentricTest extends LinearOpMode {
  private Servo magServo;
  private String colorWanted; // will be set each loop from sequence
  private String[] slots = new String[3];
  private int currentSlot = 0;

  private com.qualcomm.robotcore.util.ElapsedTime feederTimer =
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

  private int countBalls() {
    int n = 0;
    for (String s : slots) if (s != null) n++;
    return n;
  }

  // hardware/state
  private boolean feederActive = false;
  private boolean aPrev = false;

  // servo positions (tune!)
  private static final double FEED_UP_POS = 0.20;
  private static final double FEED_DOWN_POS = 0.00;

  private final double[] positions = {0.0, 0.33, 0.66}; // adjust for servo

  private ElapsedTime runtime = new ElapsedTime();
  private double lastMoveTime = 0;
  private final double MOVE_DELAY = 400; // ms
  // private boolean hasWantedColor = false;
  private boolean waitingForClear = false;
  private DcMotor frontLeft, frontRight, backLeft, backRight, flywheelMotor, intake;
  private IMU imu;
  private ColorSensor colorSensor;
  private Servo feeder;
  private DistanceSensor range;
  private Flywheel flywheel;

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
    intake = hardwareMap.get(DcMotor.class, "intake");

    // frontRight.setDirection(DcMotor.Direction.REVERSE);
    // backRight.setDirection(DcMotor.Direction.REVERSE);

    flywheel = new Flywheel((DcMotorEx) flywheelMotor);

    moveTimer.reset();
    feederTimer.reset();
    feeder.setPosition(FEED_DOWN_POS);

    // IMU
    // imu = hardwareMap.get(IMU.class, "imu");
    //  IMU.Parameters imuParams =
    //          new IMU.Parameters(
    //                 new RevHubOrientationOnRobot(
    //                          RevHubOrientationOnRobot.LogoFacingDirection.UP,
    //                          RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
    //  imu.initialize(imuParams);
    //  imu.resetYaw();

    setWantedSequence("GPP");

    waitForStart();

    while (opModeIsActive()) {

      if (runtime.milliseconds() - lastMoveTime > MOVE_DELAY) {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red * 8, green * 8, blue * 8, hsv);

        float h = hsv[0];
        float s = hsv[1];
        float v = hsv[2];

        String detected = null;

        // Detect clear gap before next ball
        if (range.getDistance(DistanceUnit.INCH) < 2.0) {
          waitingForClear = false;
        }

        if (!waitingForClear) {
          // Green (~160°)
          if (s > 0.6 && v > 40 && h >= 150 && h <= 170) {
            detected = "Green";
          }
          // Purple (~230°)
          else if (s > 0.3 && v > 40 && h >= 220 && h <= 240) {
            detected = "Purple";
          }

          if (detected != null) {
            slots[currentSlot] = detected;
            currentSlot = (currentSlot + 1) % 3;
            magServo.setPosition(positions[currentSlot]);
            lastMoveTime = runtime.milliseconds();
            waitingForClear = true;
          }
        }
      }

      // Flywheel control (replace 100 with real distance in inches)
      if (gamepad1.right_trigger > 0.25) {
        flywheel.setTargetDistance(100); // target 100 inches away
        flywheel.enable(); // let the flywheel spin up
      } else {
        flywheel.disable(); // stop th flywheel
      }

      // Which chamber is at shooter (intake at currentSlot; shooter +2 steps CW)
      int shootIdx = (currentSlot + 2) % 3;

      // Button edge
      boolean aNow = gamepad1.a;
      boolean aPressed = aNow && !aPrev;
      aPrev = aNow;

      String shootColor = slots[shootIdx];

      // Set current wanted from sequence (for checks & telemetry)
      colorWanted = currentWanted();

      // Feed if correct color AND A pressed
      if (aPressed && matchesWanted(shootColor)) {
        feeder.setPosition(FEED_UP_POS);
        feederTimer.reset();
        feederActive = true;

        // consume ball
        slots[shootIdx] = null;

        advanceWanted();
      }

      // Auto-return feeder
      if (feederActive && feederTimer.milliseconds() > FEED_HOLD_MS) {
        feeder.setPosition(FEED_DOWN_POS);
        feederActive = false;
      }
      boolean hasWantedColor =
          matchesWanted(slots[0]) || matchesWanted(slots[1]) || matchesWanted(slots[2]);
      // If wrong color at shooter, step mag with cooldown
      if ((shootColor == null && hasWantedColor)
          || (!matchesWanted(shootColor) && shootColor != null)) {
        if (moveTimer.milliseconds() > MOVE_COOLDOWN_MS) {
          currentSlot = (currentSlot + 1) % 3;
          magServo.setPosition(positions[currentSlot]);
          moveTimer.reset();
        }
      }

      // Intake logic: reverse if full, else manual
      int filled = countBalls();
      if (filled >= 3) {
        intake.setPower(-0.5); // reject extras
      } else if (gamepad1.left_trigger > 0.25) {
        intake.setPower(0.5); // intake
      } else if (gamepad1.b) {
        intake.setPower(0.0); // stop
      } else {
        intake.setPower(0.0);
      }

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

      // Telemetry
      telemetry.addData("Wanted now", colorWanted);
      telemetry.addData("Seq", wantedSeq);
      telemetry.addData("Seq idx", wantedIdx);
      telemetry.addData("Slots", "%s | %s | %s", slots[0], slots[1], slots[2]);
      telemetry.addData("ShootIdx", shootIdx);
      telemetry.addData("ShootColor", shootColor);
      telemetry.addData("ball[0]", slots[0]);
      telemetry.addData("ball[1]", slots[1]);
      telemetry.addData("ball[2]", slots[2]);
      // telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
      telemetry.update();
    }
  }
}
