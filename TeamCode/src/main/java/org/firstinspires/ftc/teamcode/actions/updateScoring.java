package org.firstinspires.ftc.teamcode.actions; // file: MySetPositionAction.java

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;

public class updateScoring implements Action {
  private final ScoringSystem scoringSystem;

  public updateScoring(ScoringSystem scoringSystem) {
    this.scoringSystem = scoringSystem;
  }

  @Override
  public boolean run(TelemetryPacket packet) {
    scoringSystem.update();
    return true;
  }
}
