package org.firstinspires.ftc.teamcode.actions; // file: MySetPositionAction.java

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;

public class ShootAction implements Action {
  private final ScoringSystem scoringSystem;

  public ShootAction(ScoringSystem scoringSystem) {
    this.scoringSystem = scoringSystem;
  }

  @Override
  public boolean run(TelemetryPacket packet) {
    return scoringSystem.shoot();
  }
}
