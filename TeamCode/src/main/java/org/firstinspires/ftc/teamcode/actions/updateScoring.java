package org.firstinspires.ftc.teamcode.actions; // file: MySetPositionAction.java

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.AutoRobot;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

public class updateScoring implements Action {
  private Telemetry telemetry;
  private final ScoringSystem scoringSystem;
  private final AutoRobot robot;
  private final SimpleTimer telemetryUpdateTimer = new SimpleTimer(0.67);

  public updateScoring(ScoringSystem scoringSystem, AutoRobot robot, Telemetry telemetry) {
    this.scoringSystem = scoringSystem;
    this.robot = robot;
    this.telemetry = telemetry;
  }

  @Override
  public boolean run(TelemetryPacket packet) {
    robot.clearSensorCache();
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
