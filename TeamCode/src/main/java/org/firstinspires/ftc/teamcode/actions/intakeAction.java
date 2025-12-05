package org.firstinspires.ftc.teamcode.actions;// file: MySetPositionAction.java

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;

public class intakeAction implements Action {
    private final ScoringSystem scoringSystem;

    public intakeAction(ScoringSystem scoringSystem) {
        this.scoringSystem = scoringSystem;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        scoringSystem.fillMag();
        return false;
    }
}
