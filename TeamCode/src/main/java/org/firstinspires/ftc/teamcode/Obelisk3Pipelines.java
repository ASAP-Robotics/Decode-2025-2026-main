package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestTeleObelisk", group = "Vision")
public class Obelisk3Pipelines extends OpMode {

  private Limelight3A limelight;

  // Only allow these Limelight pipeline INDEXES (0-9)
  private final int[] allowedPipelines = {0, 3, 4};
  private final int[] pipelineID = {21, 22, 23};
  private boolean hasValidResult = false;
  private boolean[] apriltagInSight = {false, false, false};
  private boolean[] usedPipeline = {false, false, false};

  private int pipelineIdx = 0;

  private long lastSwitchMs = 0;
  private static final long SWITCH_COOLDOWN_MS = 50;
  private double[] apriltagSize = {0.0, 0.0, 0.0};

  @Override
  public void init() {
    limelight = hardwareMap.get(Limelight3A.class, "limelight");

    limelight.setPollRateHz(100);
    limelight.start();

    telemetry.addLine("TestTeleObelisk");
    telemetry.addLine("Dpad Left/Right: switch pipelines 1 → 3 → 4");
    telemetry.addData("Requested Pipeline", allowedPipelines[pipelineIdx]);
    telemetry.update();
  }

  private int findBiggest() {
    int biggest = 0;
    for (int i = 1; i < apriltagSize.length; i++) {
      if (apriltagSize[i] > apriltagSize[biggest]) biggest = i;
    }
    return (apriltagSize[biggest] > 0.0) ? biggest : -1;
  }

  private void switchToIndex(int idx) {
    int pipeline = allowedPipelines[idx];
    limelight.pipelineSwitch(pipeline);

    usedPipeline[idx] = true; // idx is 0..2, safe
    lastSwitchMs = System.currentTimeMillis();
  }

  @Override
  public void start() {
    pipelineIdx = 0;
    switchToIndex(pipelineIdx);
  }

  @Override
  public void loop() {
    long now = System.currentTimeMillis();
    boolean canSwitch = (now - lastSwitchMs) > SWITCH_COOLDOWN_MS;

    if (canSwitch) {
      pipelineIdx = (pipelineIdx + 1) % allowedPipelines.length;
      switchToIndex(pipelineIdx);
    }

    int requested = allowedPipelines[pipelineIdx];

    LLResult result = limelight.getLatestResult();
    boolean valid = (result != null && result.isValid());

    telemetry.addData("Requested Pipeline", requested);

    if (result == null) {
      telemetry.addLine("Result: null (no data yet)");
      telemetry.update();
      return;
    }

    telemetry.addData("Result Valid", valid);
    telemetry.addData("Actual Pipeline", result.getPipelineIndex());
    telemetry.addData("Staleness (ms)", result.getStaleness());

    // Ignore stale or wrong-pipeline frames right after switching
    if (valid && result.getPipelineIndex() == requested && result.getStaleness() < 200) {
      double size = result.getTa();

      // map pipeline -> slot 0..2
      int slot = -1;
      if (requested == 0) slot = 0;
      else if (requested == 3) slot = 1;
      else if (requested == 4) slot = 2;

      if (slot != -1) {
        apriltagInSight[slot] = true;
        apriltagSize[slot] = size;
        hasValidResult = true;
      }
    }

    if (usedPipeline[0] && usedPipeline[1] && usedPipeline[2] && hasValidResult) {
      int biggest = findBiggest();
      if (biggest != -1) {
        telemetry.addLine("apriltag " + pipelineID[biggest]);
      } else {
        telemetry.addLine("No tag sizes recorded yet");
      }
    }

    telemetry.update();
  }

  @Override
  public void stop() {
    if (limelight != null) limelight.stop();
  }
}
