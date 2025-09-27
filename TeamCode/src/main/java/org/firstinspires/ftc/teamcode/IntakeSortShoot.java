package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * FULL SYSTEM (No Limelight, No Turret) - Driver toggles intake (A). Intake runs FWD until 3 balls
 * then REV to reject extras (with hysteresis). Pauses during a code run. - Magazine color sorter at
 * INTAKE with 3 slots (120° apart). Shooter FEEDER is 240° CW from intake (2 steps offset). -
 * Trigger (RT) requests ONE shot. Flywheel only spins when a shot is requested or a feed pulse is
 * finishing. - Code sequencer for PPG / PGP / GPP: press B to arm/run, D-pad Up cycles codes. Each
 * trigger press advances one step. - No camera, no turret. RPM is manual via presets + fine/coarse
 * nudges.
 */
@TeleOp(name = "Shooter – No Limelight / No Turret", group = "Competition")
public class IntakeSortShoot extends LinearOpMode {

  // ======================== CONFIG ========================
  static class CFG {
    // Intake
    static final double INTAKE_POWER_FWD = 0.80; // run until magazine full
    static final double INTAKE_POWER_REV = -0.60; // reject when full
    static final int MAG_CAPACITY = 3; // number of slots

    // Flywheel
    static final double TICKS_PER_REV = 28 * 20; // EDIT: encoder ticks per output rev
    static final boolean FLYWHEEL_REVERSED = false;
    static final double FLYWHEEL_MAX_RPM = 4500;
    static final double FLYWHEEL_RPM_TOL = 60; // +/- rpm for ready
    static final PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(20.0, 0.0, 5.0, 12.0);

    // Manual RPM presets (no camera distance)
    static final double RPM_SHORT = 1600;
    static final double RPM_MID = 2000;
    static final double RPM_LONG = 2400;
    static final int RPM_FINE_STEP = 25; // D-pad Left/Right
    static final int RPM_COARSE_STEP = 100; // D-pad Down/Up

    // Feeder
    static final double FEEDER_LOAD_POS = 0.15;
    static final double FEEDER_FIRE_POS = 0.55;
    static final long FEED_PULSE_MS = 160;
    static final long FEED_COOLDOWN_MS = 220;
  }

  // ========================================================

  // Motors/servos
  private DcMotorEx flywheel, intake;
  private Servo feeder;

  // Subsystems
  private BallSorter sorter; // magazine color sorter (intake sensor -> feeder offset)

  // Code sequence (PPG / PGP / GPP)
  private final String[] codes = {"PPG", "PGP", "GPP"};
  private int codeIndex = 0; // selects which code is active
  private boolean seqActive = false; // executing a 3-shot sequence
  private int seqStep = 0; // 0..2 index into selected code
  private boolean lastB = false; // edge detect for B press
  private boolean lastDpadUp = false; // edge detect to cycle codes

  // Driver controls
  private boolean intakeEnabled = false;
  //   private boolean lastB = false;           // B edge (intake toggle)
  private boolean triggerHeldLast = false; // RT edge detection
  private boolean flywheelEnabled = false; // RT toggles flywheel on/off

  // Intake hysteresis
  private boolean intakeFull = false;

  // State
  private final ElapsedTime time = new ElapsedTime();
  private double targetRpm = CFG.RPM_MID; // current manual target

  // Feeder non-blocking pulse state
  private enum FeedState {
    IDLE,
    OPEN,
    COOLDOWN
  }

  private FeedState feedState = FeedState.IDLE;
  private long feedStateStart = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    // Hardware map
    flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    feeder = hardwareMap.get(Servo.class, "feeder");
    try {
      intake = hardwareMap.get(DcMotorEx.class, "intake");
    } catch (Exception ignored) {
      intake = null;
    }

    // Ball sorter (sensor @ intake; FEEDER is 240° clockwise => 2 steps ahead)
    try {
      DistanceSensor range = hardwareMap.get(DistanceSensor.class, "colorSensor");
      ColorSensor color = hardwareMap.get(ColorSensor.class, "colorSensor");
      Servo magServo = hardwareMap.get(Servo.class, "magServo");
      double[] magPos = {0.00, 0.33, 0.66}; // EDIT to your mechanism
      sorter = new BallSorter(range, color, magServo, magPos, /*feederOffsetSteps*/ 2);
    } catch (Exception ignored) {
      sorter = null;
    }

    // Directions & modes
    if (CFG.FLYWHEEL_REVERSED) flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
    flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, CFG.FLYWHEEL_PIDF);

    feeder.setPosition(CFG.FEEDER_LOAD_POS);
    time.reset();
    telemetry.addLine(
        "Ready | B=toggle intake | RT=toggle flywheel | LB=start code | D-pad Up=cycle code |"
            + " X=short Y=mid RB=long | D-pad L/R=±25, D/U=±100 RPM");
    telemetry.update();
    waitForStart();

    double lastTime = time.seconds();

    while (opModeIsActive()) {

      double now = time.seconds();
      double dt = Math.max(1e-3, Math.min(0.1, now - lastTime));
      lastTime = now;

      // 0) Update ball sorter (keeps slot database fresh)
      if (sorter != null) sorter.update();

      // ===== DRIVER CONTROLS =====
      boolean lbNow = gamepad1.left_bumper;
      boolean rTrigNow = gamepad1.right_trigger > 0.5;
      boolean bNow = gamepad1.b;
      boolean upNow = gamepad1.dpad_up;
      boolean dnNow = gamepad1.dpad_down;
      boolean leftNow = gamepad1.dpad_left;
      boolean rightNow = gamepad1.dpad_right;
      boolean xNow = gamepad1.x;
      boolean yNow = gamepad1.y;
      boolean rbNow = gamepad1.right_bumper; // use RB for LONG preset to avoid B conflict

      // Intake toggle (B)
      // boolean bNow = gamepad1.b;
      if (bNow && !lastB) intakeEnabled = !intakeEnabled;
      lastB = bNow;

      // Flywheel toggle with Right Trigger (rising edge)
      if (rTrigNow && !triggerHeldLast) flywheelEnabled = !flywheelEnabled;
      triggerHeldLast = rTrigNow;

      // Code selection and start
      if (upNow && !lastDpadUp) codeIndex = (codeIndex + 1) % codes.length;
      if (lbNow && !seqActive) {
        seqActive = true;
        seqStep = 0;
      }
      lastDpadUp = upNow;

      // Manual RPM presets (no camera)
      if (xNow) targetRpm = CFG.RPM_SHORT;
      if (yNow) targetRpm = CFG.RPM_MID;
      if (rbNow) targetRpm = CFG.RPM_LONG; // moved LONG to RB to avoid clash with B

      // Fine/coarse nudges
      if (leftNow) targetRpm = clip(targetRpm - CFG.RPM_FINE_STEP, 0, CFG.FLYWHEEL_MAX_RPM);
      if (rightNow) targetRpm = clip(targetRpm + CFG.RPM_FINE_STEP, 0, CFG.FLYWHEEL_MAX_RPM);
      if (dnNow) targetRpm = clip(targetRpm - CFG.RPM_COARSE_STEP, 0, CFG.FLYWHEEL_MAX_RPM);
      if (upNow && !lastDpadUp) {
        /* already used to cycle code; ignore coarse + */
      }

      // ===== INTAKE =====
      if (intake != null) {
        int count = (sorter != null) ? sorter.ballCount() : 0;
        if (!intakeFull && count >= CFG.MAG_CAPACITY) intakeFull = true;
        if (intakeFull && count <= CFG.MAG_CAPACITY - 1) intakeFull = false;
        double pwr = 0.0;
        if (!seqActive && intakeEnabled) {
          pwr = intakeFull ? CFG.INTAKE_POWER_REV : CFG.INTAKE_POWER_FWD;
        } else {
          pwr = 0.0; // off during sequence
        }
        intake.setPower(pwr);
      }

      // Update feeder pulse state machine every loop
      updateFeeder();

      // ===== FLYWHEEL (toggle + auto during feed/sequence) =====
      boolean keepSpinning = flywheelEnabled || (feedState != FeedState.IDLE) || seqActive;
      if (keepSpinning) flywheel.setVelocity(rpmToTicksPerSec(targetRpm));
      else flywheel.setVelocity(0);
      boolean rpmReady =
          Math.abs(getCurrentRpm() - (keepSpinning ? targetRpm : 0)) <= CFG.FLYWHEEL_RPM_TOL
              && (keepSpinning ? targetRpm > 100 : true);

      // ===== SHOOTING =====
      if (seqActive && sorter != null) {
        String code = codes[codeIndex];
        if (seqStep < 3) {
          char wantChar = code.charAt(seqStep);
          String want = (wantChar == 'P') ? "Purple" : "Green";

          // Ensure desired color is under the FEEDER
          sorter.seekColorAtFeeder(want);

          // Auto-fire when at speed, correct color under FEEDER, and feeder idle
          boolean colorReady = sorter.feederHas(want);
          if (rpmReady && colorReady && feedState == FeedState.IDLE) {
            startFeedPulse();
            sorter.consumeFeeder();
            seqStep++;
          }
        } else {
          // finished all three shots
          seqActive = false;
        }
      }

      // ===== TELEMETRY =====
      telemetry.addData(
          "code", codes[codeIndex] + (seqActive ? " [RUN] step=" + seqStep : " [ARMED]"));
      telemetry.addData("balls", sorter != null ? sorter.ballCount() : -1);
      telemetry.addData("targetRPM", Math.round(targetRpm));
      telemetry.addData("nowRPM", Math.round(getCurrentRpm()));
      telemetry.addData("intakeEnabled", intakeEnabled);
      telemetry.addData("flywheelEnabled", flywheelEnabled);
      telemetry.update();
    }
  }

  // =================== ACTUATION HELPERS ===================
  private void startFeedPulse() {
    if (feedState == FeedState.IDLE) {
      feeder.setPosition(CFG.FEEDER_FIRE_POS);
      feedState = FeedState.OPEN;
      feedStateStart = (long) time.milliseconds();
    }
  }

  private void updateFeeder() {
    long ms = (long) time.milliseconds();
    switch (feedState) {
      case OPEN:
        if (ms - feedStateStart >= CFG.FEED_PULSE_MS) {
          feeder.setPosition(CFG.FEEDER_LOAD_POS);
          feedState = FeedState.COOLDOWN;
          feedStateStart = ms;
        }
        break;
      case COOLDOWN:
        if (ms - feedStateStart >= CFG.FEED_COOLDOWN_MS) {
          feedState = FeedState.IDLE;
        }
        break;
      case IDLE:
      default:
        break;
    }
  }

  private static double clip(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  private double rpmToTicksPerSec(double rpm) {
    return rpm * CFG.TICKS_PER_REV / 60.0;
  }

  private double getCurrentRpm() {
    return flywheel.getVelocity() * 60.0 / CFG.TICKS_PER_REV;
  }

  // =================== BALL SORTER (HSV, intake sensor -> feeder offset) ===================
  static class BallSorter {
    final DistanceSensor range;
    final ColorSensor color;
    final Servo mag;
    final double[] positions; // servo positions for slots 0..2
    final ElapsedTime time = new ElapsedTime();

    final String[] slots = new String[3];
    int currentSlot = 0; // slot aligned to the INTAKE SENSOR
    final int feederOffsetSteps; // steps from sensor to FEEDER (0..2)

    // Tuning
    long MOVE_DELAY_MS = 300; // minimum time between moves
    boolean waitingForClear = false; // require grey/low‑sat before reading a new color

    // HSV thresholds (EDIT to your lighting)
    float S_GREY_MAX = 0.20f; // below this is greyish
    float V_GREY_MAX = 0.35f; // 0..1 range for Color.RGBToHSV on Android
    float S_MIN_PURPLE = 0.30f, V_MIN_PURPLE = 0.40f, H_MIN_PURPLE = 220f, H_MAX_PURPLE = 240f;
    float S_MIN_GREEN = 0.60f, V_MIN_GREEN = 0.40f, H_MIN_GREEN = 150f, H_MAX_GREEN = 170f;

    long lastMoveMs = 0;

    BallSorter(ColorSensor color, Servo mag, double[] slotPositions, DistanceSensor range) {
      this(range, color, mag, slotPositions, 0);
    }

    BallSorter(
        DistanceSensor range,
        ColorSensor color,
        Servo mag,
        double[] slotPositions,
        int feederOffsetSteps) {
      this.range = range;
      this.color = color;
      this.mag = mag;
      this.positions = slotPositions.clone();
      this.feederOffsetSteps = ((feederOffsetSteps % 3) + 3) % 3; // normalize 0..2
      mag.setPosition(positions[currentSlot]);

      time.reset();
    }

    /** Call periodically to sample the sensor and update slots. */
    void update() {
      double cm = range.getDistance(DistanceUnit.CM);
      long now = (long) (time.milliseconds());
      if (now - lastMoveMs < MOVE_DELAY_MS) return; // debounce

      int r = color.red();
      int g = color.green();
      int b = color.blue();
      float[] hsv = new float[3];
      android.graphics.Color.RGBToHSV(r * 8, g * 8, b * 8, hsv);
      float h = hsv[0], s = hsv[1], v = hsv[2];

      // clear gate on grey/low value
      if (cm > 5) waitingForClear = false;

      if (!waitingForClear) {
        String detected = null;
        if (s > S_MIN_GREEN && v > V_MIN_GREEN && h >= H_MIN_GREEN && h <= H_MAX_GREEN) {
          detected = "Green";
        } else if (s > S_MIN_PURPLE && v > V_MIN_PURPLE && h >= H_MIN_PURPLE && h <= H_MAX_PURPLE) {
          detected = "Purple";
        }
        if (detected != null) {
          slots[currentSlot] = detected;
          stepToNextSlot();
          waitingForClear = true;
        }
      }
    }

    /** Rotate servo to next slot index (0->1->2->0). */
    void stepToNextSlot() {
      currentSlot = (currentSlot + 1) % 3;
      mag.setPosition(positions[currentSlot]);
      lastMoveMs = (long) time.milliseconds();
    }

    /** Index of the slot currently under the FEEDER (shooter). */
    int feederIndex() {
      return (currentSlot + feederOffsetSteps) % 3;
    }

    /** Color currently under the FEEDER. */
    String feederColor() {
      return slots[feederIndex()];
    }

    /** True if desired color is under FEEDER now. */
    boolean feederHas(String wanted) {
      return wanted != null && wanted.equals(feederColor());
    }

    /** Rotate until desired color is under FEEDER (non-blocking; call each loop). */
    void seekColorAtFeeder(String wanted) {
      if (wanted == null) return;
      if (wanted.equals(feederColor())) return; // already there
      // Try moving one step toward the slot that will put 'wanted' under FEEDER
      for (int i = 1; i <= 2; i++) {
        int candidateCurrent = (currentSlot + i) % 3; // where sensor would be after i steps
        int candidateFeeder = (candidateCurrent + feederOffsetSteps) % 3;
        if (wanted.equals(slots[candidateFeeder])) {
          stepToNextSlot();
          break;
        }
      }
    }

    /** After shooting the FEEDER slot, clear it. */
    void consumeFeeder() {
      slots[feederIndex()] = null;
    }

    /** Number of non-empty slots currently stored. */
    int ballCount() {
      int c = 0;
      for (String s : slots) if (s != null) c++;
      return c;
    }
  }
}
