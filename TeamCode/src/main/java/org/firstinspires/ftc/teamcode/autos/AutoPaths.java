package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.AutoEndShutdowAction;
import org.firstinspires.ftc.teamcode.actions.setAiming;
import org.firstinspires.ftc.teamcode.actions.setScoringPose;
import org.firstinspires.ftc.teamcode.actions.shootAction;
import org.firstinspires.ftc.teamcode.actions.updateScoring;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

public class AutoPaths {
    private int flipy = 1;
    private AllianceColor allianceColor;
    AutoPaths(AllianceColor color){
        allianceColor = color;
        if (allianceColor == AllianceColor.RED) {
            flipy = -1;
        }
    }

    ParallelAction getFarSideAuto(ScoringSystem scoringSystem, MecanumDrive drive){
        return new ParallelAction( // BIGGEST BOI
                new updateScoring(scoringSystem, telemetry),
                // new updateTelemetry(telemetry),
                new SequentialAction( // BIG BOI
                         // 1
                        new setAiming(134, -12,3000,30, flipy, scoringSystem),
                        new shootAction(scoringSystem),
                        new SequentialAction( // shoot 1
                                drive
                                        .actionBuilder(new Pose2d(63,-8.6*flipy,Math.toRadians(-90)*flipy))
                                        .strafeToLinearHeading(
                                                new Pose2d(62, -61 * flipy, flipy * (Math.toRadians(-90))).position,
                                                Math.toRadians(-90)*flipy,
                                                new TranslationalVelConstraint(200.0),
                                                new ProfileAccelConstraint(-30, 200))
                                        .waitSeconds(0.2)
                                        // shoot 2
                                        .strafeToLinearHeading(new Vector2d(63,-15*flipy),flipy * (Math.toRadians(-90)),
                                                new TranslationalVelConstraint(250.0),
                                                new ProfileAccelConstraint(-50, 200))
                                        .build(),
                                new shootAction(scoringSystem)),
                        new AutoEndShutdowAction(scoringSystem)));
    }
    ParallelAction getCloseSide15Auto(ScoringSystem scoringSystem, MecanumDrive drive){
        double secondShootX = -5.5;
        double secondShootY = -17;
        return new ParallelAction( // BIGGEST BOI
                new updateScoring(scoringSystem, telemetry),
                // new updateTelemetry(telemetry),
                new SequentialAction( // BIG BOI
                        new SequentialAction(new setScoringPose(scoringSystem, allianceColor)), // 1
                        //new ObeliskSearch(limelight, telemetry),
                        new setAiming(30, -65,1900, 70, flipy, scoringSystem),
                        new shootAction(scoringSystem),
                        new setAiming(90, -49,2600, 35, flipy,  scoringSystem),
                        new SequentialAction( // shoot 1
                                drive
                                        .actionBuilder(allianceColor.getAutoStartPosition())
                                        // -----SHOOT1------\\
                                        .splineToLinearHeading(
                                                allianceColor.getAutoRRShootPosition(),
                                                (Math.PI / -8) * flipy,
                                                new TranslationalVelConstraint(250.0),
                                                new ProfileAccelConstraint(-50, 240))
                                        .splineToLinearHeading(
                                                new Pose2d(14, -27.9 * flipy, flipy * (Math.toRadians(-90))),
                                                (Math.PI / -2) * flipy,
                                                new TranslationalVelConstraint(250.0),
                                                new ProfileAccelConstraint(-50, 240))
                                        .strafeTo(
                                                new Pose2d(15, -61.5 * flipy, flipy * (Math.toRadians(-90))).position,
                                                new TranslationalVelConstraint(250.0),
                                                new ProfileAccelConstraint(-30, 210))
                                        .waitSeconds(0.2)
                                        // shoot 2
                                        .strafeToLinearHeading(new Vector2d(secondShootX,secondShootY*flipy),flipy * (Math.toRadians(-90)),new TranslationalVelConstraint(250.0),
                                                new ProfileAccelConstraint(-50, 240))
                                        .build(),
                                new shootAction(scoringSystem)),

                        new SequentialAction( // pickup2 (gate pickup)
                                //GATE PICKUP
                                drive
                                        .actionBuilder((new Pose2d(secondShootX,secondShootY*flipy,flipy * (Math.toRadians(-90)))))
                                        .splineToLinearHeading(
                                                new Pose2d(15, -30 * flipy, flipy * Math.toRadians(-110)),
                                                flipy * Math.PI/ -1,
                                                new TranslationalVelConstraint(250.0),
                                                new ProfileAccelConstraint(-50, 240))
                                        .splineToLinearHeading(
                                                new Pose2d(13.5, -62 * flipy, flipy * Math.toRadians(-110)),
                                                flipy * Math.PI/ -1,
                                                new TranslationalVelConstraint(250.0),
                                                new ProfileAccelConstraint(-50, 240))
                                        //GATE PICKUP
                                        .waitSeconds(1.5)
                                        // shoot3
                                        .strafeToLinearHeading(new Vector2d(secondShootX,secondShootY*flipy),flipy * (Math.toRadians(-90)),new TranslationalVelConstraint(250.0),
                                                new ProfileAccelConstraint(-50, 240))
                                        .build(),
                                new shootAction(scoringSystem)),
                        new SequentialAction(

                                drive
                                        .actionBuilder(new Pose2d(secondShootX,secondShootY*flipy,flipy * (Math.toRadians(-90))))
                                        .splineToLinearHeading(
                                                new Pose2d(43.1, -27 * flipy, flipy * (Math.toRadians(-90))),
                                                (Math.PI / -2) * flipy,
                                                new TranslationalVelConstraint(250.0),
                                                new ProfileAccelConstraint(-50, 240))
                                        // pickup 4 gpp
                                        .splineToLinearHeading(
                                                new Pose2d(40.1, -62.5 * flipy, flipy * (Math.toRadians(-90))),
                                                flipy * (Math.PI/ -2)
                                        )
                                        // shoot4
                                        .strafeToLinearHeading(new Vector2d(secondShootX,secondShootY*flipy),flipy * (Math.toRadians(-90)),new TranslationalVelConstraint(250.0),
                                                new ProfileAccelConstraint(-50, 240))
                                        .build(),
                                new shootAction(scoringSystem),
                                new setAiming(70, -30,2500,32, flipy, scoringSystem)),
                        new SequentialAction( // pickup close
                                drive
                                        .actionBuilder((new Pose2d(secondShootX,secondShootY*flipy,flipy * (Math.toRadians(-90)))))
                                        // go to pickup 4
                                        .splineToLinearHeading(
                                                // pickup PPG first slot
                                                new Pose2d(-13.75, -49 * flipy, flipy * (Math.toRadians(-90))),
                                                (Math.PI / -2) * flipy,
                                                new TranslationalVelConstraint(180.0),
                                                new ProfileAccelConstraint(-50, 240))
                                        .waitSeconds(.8)
                                        // shoot 5 and leave3
                                        .strafeToLinearHeading(
                                                new Pose2d(secondShootX-30,secondShootY*flipy,flipy * (Math.toRadians(-90))).position,
                                                flipy * (Math.toRadians(-90)),
                                                new TranslationalVelConstraint(300.0),
                                                new ProfileAccelConstraint(-50, 240+10))
                                        .build(),

                                new shootAction(scoringSystem)),
                        new AutoEndShutdowAction(scoringSystem)));
    }
}
