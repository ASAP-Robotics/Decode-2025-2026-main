package org.firstinspires.ftc.teamcode.actions; // file: MySetPositionAction.java

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;

public class setAiming implements Action {
  private final ScoringSystem scoringSystem;
  private double angle;
  private double distance;
  private int flipy;

  public setAiming(double distance, double angle, int flipy, ScoringSystem scoringSystem) {
    this.scoringSystem = scoringSystem;
    this.distance = distance;
    this.angle = angle;
    this.flipy = flipy;
  }

  @Override
  public boolean run(TelemetryPacket packet) {
    scoringSystem.overrideAiming(this.distance, this.flipy * this.angle);
    scoringSystem.update(true);
    return false;
  }
}
