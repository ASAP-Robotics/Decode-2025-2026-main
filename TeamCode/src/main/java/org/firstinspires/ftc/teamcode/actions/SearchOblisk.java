package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;

public class SearchOblisk implements Action {
  protected ScoringSystem scoringSystem;

  public SearchOblisk(ScoringSystem scoringSystem) {
    this.scoringSystem = scoringSystem;
  }

  @Override
  public boolean run(@NonNull TelemetryPacket telemetryPacket) {
    return scoringSystem.getLimelightState() != Limelight.LimeLightMode.NAVIGATION;
  }
}
