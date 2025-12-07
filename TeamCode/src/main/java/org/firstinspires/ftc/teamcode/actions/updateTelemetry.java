package org.firstinspires.ftc.teamcode.actions; // file: MySetPositionAction.java

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class updateTelemetry implements Action {
  private final Telemetry telemetry;

  public updateTelemetry(Telemetry telemetry) {
    this.telemetry = telemetry;
  }

  @Override
  public boolean run(TelemetryPacket packet) {
    telemetry.update();
    return true;
  }
}
