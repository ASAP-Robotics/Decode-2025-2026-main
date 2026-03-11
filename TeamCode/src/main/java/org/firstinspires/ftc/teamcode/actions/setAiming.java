package org.firstinspires.ftc.teamcode.actions; // file: MySetPositionAction.java

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;

public class setAiming implements Action {
  private final ScoringSystem scoringSystem;
  private final double angle;
  private final double distance;
  private final int flipY;
  private final int rpm;
  private final int hoodAngle;

  public setAiming(
      double distance,
      double angle,
      int rpm,
      int hoodAngle,
      int flipY,
      ScoringSystem scoringSystem) {
    this.scoringSystem = scoringSystem;
    this.distance = distance;
    this.angle = angle;
    this.flipY = flipY;
    this.rpm = rpm;
    this.hoodAngle = hoodAngle;
  }

  @Override
  public boolean run(@NonNull TelemetryPacket packet) {
    scoringSystem.tuneAiming(this.rpm, this.hoodAngle);
    scoringSystem.overrideAiming(this.distance, this.flipY * this.angle);
    scoringSystem.update(true);
    return false;
  }
}
