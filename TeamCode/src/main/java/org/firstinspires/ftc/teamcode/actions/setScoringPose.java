package org.firstinspires.ftc.teamcode.actions; // file: MySetPositionAction.java

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;

public class setScoringPose implements Action {
  private final ScoringSystem scoringSystem;

  public setScoringPose(ScoringSystem scoringSystem) {
    this.scoringSystem = scoringSystem;
  }

  @Override
  public boolean run(TelemetryPacket packet) {

    scoringSystem.setRobotPosition(new Pose2D(DistanceUnit.INCH, -2, 31, AngleUnit.DEGREES, 90));
    scoringSystem.update();
    return false;
  }
}
