package org.firstinspires.ftc.teamcode.actions; // file: MySetPositionAction.java


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

public class setScoringPose implements Action {
  private final ScoringSystem scoringSystem;
  private final AllianceColor allianceColor;

  public setScoringPose(ScoringSystem scoringSystem, AllianceColor allianceColor) {
    this.scoringSystem = scoringSystem;
    this.allianceColor = allianceColor;
  }

  @Override
  public boolean run(TelemetryPacket packet) {

    scoringSystem.setRobotPosition(allianceColor.getAutoSSShootPosition());
    scoringSystem.update();
    return false;
  }
}
