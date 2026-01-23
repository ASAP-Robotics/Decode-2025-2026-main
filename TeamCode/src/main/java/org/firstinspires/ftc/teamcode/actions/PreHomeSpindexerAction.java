package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;

/**
 * Action to move spindexer at the end of Auto so homing at the start of TeleOp will be fast
 */
public class PreHomeSpindexerAction implements Action {
  protected ScoringSystem scoringSystem;

  public PreHomeSpindexerAction(ScoringSystem scoringSystem) {
    this.scoringSystem = scoringSystem;
  }

  @Override
  public boolean run(@NonNull TelemetryPacket telemetryPacket) {
    scoringSystem.homeSpindexer();
    return false;
  }
}
