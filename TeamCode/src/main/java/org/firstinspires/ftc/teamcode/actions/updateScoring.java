package org.firstinspires.ftc.teamcode.actions; // file: MySetPositionAction.java

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

public class updateScoring implements Action {
  private final Telemetry telemetry;
  private final ScoringSystem scoringSystem;
  private final SimpleTimer telemetryUpdateTimer = new SimpleTimer(0.67);

  public updateScoring(ScoringSystem scoringSystem, Telemetry telemetry) {
    this.scoringSystem = scoringSystem;
    this.telemetry = telemetry;
  }

  @Override
  public boolean run(TelemetryPacket packet) {
    boolean update = false;
    if (!telemetryUpdateTimer.isRunning()) {
      update = true;
      telemetryUpdateTimer.start();
    }
    scoringSystem.update(update);
    if (update) telemetry.update();
    return true;
  }
}
