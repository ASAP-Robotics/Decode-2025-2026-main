package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utils.PositionFileWriter;

/** Action to save the position of the robot so that TeleOp knows where it starts at */
public class SavePosition implements Action {
  private final Pose2d position;

  public SavePosition(Pose2d position) {
    this.position = position;
  }

  public boolean run(TelemetryPacket packet) {
    new PositionFileWriter().writePosition(position);
    return false;
  }
}
