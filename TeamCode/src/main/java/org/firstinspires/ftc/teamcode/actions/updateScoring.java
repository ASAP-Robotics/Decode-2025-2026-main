package org.firstinspires.ftc.teamcode.actions; // file: MySetPositionAction.java

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.AutoRobot;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;

public class updateScoring implements Action {
  private final ScoringSystem scoringSystem;
  private final AutoRobot robot;

  public updateScoring(ScoringSystem scoringSystem, AutoRobot robot) {
    this.scoringSystem = scoringSystem;
    this.robot = robot;
  }

  @Override
  public boolean run(TelemetryPacket packet) {
    robot.clearSensorCache();
    scoringSystem.update(false);
    return true;
  }
}
