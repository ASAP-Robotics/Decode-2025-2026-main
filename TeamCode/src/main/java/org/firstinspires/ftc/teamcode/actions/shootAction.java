package org.firstinspires.ftc.teamcode.actions;// file: MySetPositionAction.java

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;

public class shootAction implements Action {
    private final ScoringSystem scoringSystem;

    public shootAction(ScoringSystem scoringSystem) {
        this.scoringSystem = scoringSystem;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        return scoringSystem.shootHalfSorted();
    }
}
