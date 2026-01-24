package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.sensors.Limelight;

public class SearchObelisk implements Action {
  protected ScoringSystem scoringSystem;

  public SearchObelisk(ScoringSystem scoringSystem) {
    this.scoringSystem = scoringSystem;
  }

  @Override
  public boolean run(@NonNull TelemetryPacket telemetryPacket) {
    return scoringSystem.getLimelightState() != Limelight.LimeLightMode.NAVIGATION;
  }
}
