package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.AutoEndShutdowAction;
import org.firstinspires.ftc.teamcode.actions.SetShootingMode;
import org.firstinspires.ftc.teamcode.actions.setAiming;
import org.firstinspires.ftc.teamcode.actions.setScoringPose;
import org.firstinspires.ftc.teamcode.actions.ShootAction;
import org.firstinspires.ftc.teamcode.actions.updateScoring;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.Spindex;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

public class AutoPaths {
  private int flipy = 1;
  private final AllianceColor allianceColor;
  private int angleOffset = 0;

  AutoPaths(AllianceColor color) {
    allianceColor = color;
    if (allianceColor == AllianceColor.RED) {
      flipy = -1;
      angleOffset = 3;
    }
  }



  ParallelAction getFarSideAuto(
      ScoringSystem scoringSystem, MecanumDrive drive, Telemetry telemetry) {
    return new ParallelAction( // BIGGEST BOI
        new updateScoring(scoringSystem, telemetry),
        // new updateTelemetry(telemetry),
        new SequentialAction( // BIG BOI
            new setAiming(122, -65-angleOffset, 3030, 35, flipy, scoringSystem), // 1
            new SequentialAction( // shoot 1
                new ShootAction(scoringSystem)),
                new setAiming(122, -73-angleOffset, 2990, 36, flipy, scoringSystem),
            new SequentialAction( // shoot 2
                drive
                    .actionBuilder(new Pose2d(62.8, -10.5 * flipy, Math.toRadians(-90) * flipy))
                    .splineToLinearHeading(
                        new Pose2d(35, -30 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -4) * flipy,
                        new TranslationalVelConstraint(160.0),
                        new ProfileAccelConstraint(-60, 130))
                        .splineToLinearHeading(
                                new Pose2d(35, -54 * flipy, flipy * (Math.toRadians(-90))),
                                (Math.PI / -4) * flipy,
                                new TranslationalVelConstraint(110.0),
                                new ProfileAccelConstraint(-60, 130))
                    .waitSeconds(0.55)
                    // shoot 2
                    .strafeToLinearHeading(
                        new Vector2d(62.8, -28 * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(160.0),
                        new ProfileAccelConstraint(-60, 130))
                    .build(),
                new ShootAction(scoringSystem)),
                new SequentialAction( // shoot 3
                        drive
                                .actionBuilder(new Pose2d(62.8, -28 * flipy, Math.toRadians(-90) * flipy))
                                .splineToLinearHeading(
                                        new Pose2d(62.8, -61 * flipy, flipy * (Math.toRadians(-90))),
                                        (Math.PI / -2) * flipy,
                                        new TranslationalVelConstraint(110.0),
                                        new ProfileAccelConstraint(-30, 50))
                                .waitSeconds(0.55)
                                // shoot 3
                                .strafeToLinearHeading(
                                        new Vector2d(62.8, -28 * flipy),
                                        flipy * (Math.toRadians(-90)),
                                        new TranslationalVelConstraint(160.0),
                                        new ProfileAccelConstraint(-30, 80))
                                .build(),
                        new ShootAction(scoringSystem)),
                new SequentialAction( // shoot 3
                        drive
                                .actionBuilder(new Pose2d(62.8, -28 * flipy, Math.toRadians(-90) * flipy))
                                .splineToLinearHeading(
                                        new Pose2d(62.8, -61 * flipy, flipy * (Math.toRadians(-90))),
                                        (Math.PI / -2) * flipy,
                                        new TranslationalVelConstraint(70.0),
                                        new ProfileAccelConstraint(-30, 80))
                                .waitSeconds(0.55)
                                // shoot 3
                                .strafeToLinearHeading(
                                        new Vector2d(62.8, -28 * flipy),
                                        flipy * (Math.toRadians(-90)),
                                        new TranslationalVelConstraint(160.0),
                                        new ProfileAccelConstraint(-30, 80))
                                .build(),
                        new ShootAction(scoringSystem)),
                new SequentialAction( // shoot 3
                        drive
                                .actionBuilder(new Pose2d(62.8, -28 * flipy, Math.toRadians(-90) * flipy))
                                .splineToLinearHeading(
                                        new Pose2d(62.8, -61 * flipy, flipy * (Math.toRadians(-90))),
                                        (Math.PI / -2) * flipy,
                                        new TranslationalVelConstraint(70.0),
                                        new ProfileAccelConstraint(-30, 80))
                                .waitSeconds(0.55)
                                // shoot 3
                                .strafeToLinearHeading(
                                        new Vector2d(62.8, -28 * flipy),
                                        flipy * (Math.toRadians(-90)),
                                        new TranslationalVelConstraint(160.0),
                                        new ProfileAccelConstraint(-30, 80))
                                .build(),
                        new ShootAction(scoringSystem)),
            new SequentialAction(
                    drive
                            .actionBuilder(new Pose2d(62.8, -28 * flipy, Math.toRadians(-90) * flipy))
                            .splineToLinearHeading(
                                    new Pose2d(62.8, -40 * flipy, flipy * (Math.toRadians(-90))),
                                    (Math.PI / -2) * flipy,
                                    new TranslationalVelConstraint(110.0),
                                    new ProfileAccelConstraint(-30, 80))
                            .build()
            ),
            new AutoEndShutdowAction(scoringSystem)));
  }

  ParallelAction getCloseSide15Auto(
      ScoringSystem scoringSystem, MecanumDrive drive, Telemetry telemetry) {
    double secondShootX = -5.5;
    double secondShootY = -17;
    final int angleOffset;
    final double xOffset = -1;
    if (allianceColor == AllianceColor.RED) {

      angleOffset = -3;
    } else {
      angleOffset = 0;
    }
    return new ParallelAction( // BIGGEST BOI
        new updateScoring(scoringSystem, telemetry),
        // new updateTelemetry(telemetry),
        new SequentialAction( // BIG BOI
            new SequentialAction(new setScoringPose(scoringSystem, allianceColor)), // 1
            // new ObeliskSearch(limelight, telemetry),
            new setAiming(90, -50.5 + angleOffset / -3, 2400, 24, flipy, scoringSystem),
            new SequentialAction( // shoot 1
                drive
                    .actionBuilder(allianceColor.getAutoStartPosition())
                    // -----SHOOT1------\\
                    .splineToLinearHeading(
                        allianceColor.getAutoRRShootPosition(),
                        (Math.PI / -8) * flipy,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 80))
                    .build(),
                new ShootAction(scoringSystem)),
            new setAiming(90, -43 + angleOffset, 2520, 26, flipy, scoringSystem),
            new SequentialAction( // pickup 1 PGP
                drive
                    .actionBuilder(allianceColor.getAutoRRShootPosition())

                    // pickup first (second slot)
                    .splineToLinearHeading(
                        new Pose2d(14, -27.9 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -2) * flipy,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 200))
                    .splineToLinearHeading(
                        new Pose2d(14.25, -59 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -2) * flipy,
                        new TranslationalVelConstraint(180.0),
                        new ProfileAccelConstraint(-30, 140))
                    .waitSeconds(0.05)
                    // shoot 2
                    .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem)),
            new SequentialAction( // pickup2 (gate pickup)
                // GATE PICKUP
                drive
                    .actionBuilder(
                        (new Pose2d(
                            secondShootX, secondShootY * flipy, flipy * (Math.toRadians(-90)))))
                    .splineToLinearHeading(
                        new Pose2d(15, -30 * flipy, flipy * Math.toRadians(-90)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .splineToLinearHeading(
                        new Pose2d(13, -59.5 * flipy, flipy * Math.toRadians(-120)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .waitSeconds(.65)
                    .strafeToLinearHeading(
                        new Vector2d(13.25, -55 * flipy), flipy * Math.toRadians(-120))
                    .strafeToLinearHeading(
                        new Pose2d(16 + xOffset * flipy, -62.5 * flipy, flipy * Math.toRadians(-90))
                            .position,
                        flipy * Math.toRadians(-90),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    // GATE PICKUP
                    .waitSeconds(.19)
                    // shoot3
                    .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem)),
            new SequentialAction(
                drive
                    .actionBuilder(
                        new Pose2d(
                            secondShootX, secondShootY * flipy, flipy * (Math.toRadians(-90))))
                    .splineToLinearHeading(
                        new Pose2d(43.1, -27 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -2) * flipy,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    // pickup 3 gpp
                    .splineToLinearHeading(
                        new Pose2d(38.1, -60 * flipy, flipy * (Math.toRadians(-90))),
                        flipy * (Math.PI / -2))
                    // shoot4
                    .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem),
                new setAiming(70, -28 + angleOffset, 2500, 29, flipy, scoringSystem)),
            new SequentialAction( // pickup close
                drive
                    .actionBuilder(
                        (new Pose2d(
                            secondShootX, secondShootY * flipy, flipy * (Math.toRadians(-90)))))
                    // go to pickup 4
                    .splineToLinearHeading(
                        // pickup PPG first slot
                        new Pose2d(
                            -16 + xOffset * flipy * 2, -51 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -3) * flipy,
                        new TranslationalVelConstraint(150.0),
                        new ProfileAccelConstraint(-50, 180))
                    .waitSeconds(.1)
                    // shoot 5 and leave3
                    .strafeToLinearHeading(
                        new Pose2d(
                                secondShootX - 30,
                                secondShootY * flipy,
                                flipy * (Math.toRadians(-90)))
                            .position,
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(300.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem)),
            new AutoEndShutdowAction(scoringSystem)));
  }

  ParallelAction getCloseSide12Auto(
      ScoringSystem scoringSystem, MecanumDrive drive, Telemetry telemetry) {
    double secondShootX = -5.5;
    double secondShootY = -17;
    final int angleOffset;
    final double xOffset = -1;
    if (allianceColor == AllianceColor.RED) {

      angleOffset = -3;
    } else {
      angleOffset = 0;
    }
    return new ParallelAction( // BIGGEST BOI
        new updateScoring(scoringSystem, telemetry),
        // new updateTelemetry(telemetry),
        new SequentialAction( // BIG BOI
            new SequentialAction(new setScoringPose(scoringSystem, allianceColor)), // 1
            // new ObeliskSearch(limelight, telemetry),
            new setAiming(90, -50.5 + angleOffset / -3, 2400, 24, flipy, scoringSystem),
            new SequentialAction( // shoot 1
                drive
                    .actionBuilder(allianceColor.getAutoStartPosition())
                    // -----SHOOT1------\\
                    .splineToLinearHeading(
                        allianceColor.getAutoRRShootPosition(),
                        (Math.PI / -8) * flipy,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 80))
                    .build(),
                new ShootAction(scoringSystem)),
            new setAiming(90, -43 + angleOffset, 2520, 26, flipy, scoringSystem),
            new SequentialAction( // pickup 1 PGP
                drive
                    .actionBuilder(allianceColor.getAutoRRShootPosition())

                    // pickup first (second slot)
                    .splineToLinearHeading(
                        new Pose2d(14, -27.9 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -2) * flipy,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 200))
                    .splineToLinearHeading(
                        new Pose2d(14.25, -59 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -2) * flipy,
                        new TranslationalVelConstraint(180.0),
                        new ProfileAccelConstraint(-30, 140))
                    .waitSeconds(0.05)
                    .strafeToLinearHeading(
                        new Pose2d(11, -53 * flipy, flipy * (Math.toRadians(-90))).position,
                        flipy * (Math.toRadians(-90)))
                    .strafeToLinearHeading(
                        new Pose2d(4, -58 * flipy, flipy * (Math.toRadians(-90))).position,
                        flipy * (Math.toRadians(-90)))
                    .strafeToLinearHeading(
                        new Pose2d(4, -30 * flipy, flipy * (Math.toRadians(-90))).position,
                        flipy * (Math.toRadians(-90)))
                    // shoot 2
                    .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem)),
            new SequentialAction( // pickup2 (gate pickup)
                // GATE PICKUP
                drive
                    .actionBuilder(
                        (new Pose2d(
                            secondShootX, secondShootY * flipy, flipy * (Math.toRadians(-90)))))
                    .splineToLinearHeading(
                        new Pose2d(15, -30 * flipy, flipy * Math.toRadians(-90)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .splineToLinearHeading(
                        new Pose2d(13, -59.5 * flipy, flipy * Math.toRadians(-120)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .waitSeconds(.65)
                    .strafeToLinearHeading(
                        new Vector2d(13.25, -55 * flipy), flipy * Math.toRadians(-120))
                    .strafeToLinearHeading(
                        new Pose2d(16 + xOffset * flipy, -62.5 * flipy, flipy * Math.toRadians(-90))
                            .position,
                        flipy * Math.toRadians(-90),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    // GATE PICKUP
                    .waitSeconds(.19)
                    // shoot3
                    .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem),
                new setAiming(70, -25 + angleOffset, 2475, 27, flipy, scoringSystem)),
            new SequentialAction( // pickup close
                drive
                    .actionBuilder(
                        (new Pose2d(
                            secondShootX, secondShootY * flipy, flipy * (Math.toRadians(-90)))))
                    // go to pickup 4
                    .splineToLinearHeading(
                        // pickup PPG first slot
                        new Pose2d(
                            -16 + xOffset * flipy * 2, -51 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -3) * flipy,
                        new TranslationalVelConstraint(150.0),
                        new ProfileAccelConstraint(-50, 180))
                    .waitSeconds(.1)
                    // shoot 5 and leave3
                    .strafeToLinearHeading(
                        new Pose2d(
                                secondShootX - 30,
                                secondShootY * flipy,
                                flipy * (Math.toRadians(-90)))
                            .position,
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(300.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem)),
            new AutoEndShutdowAction(scoringSystem)));
  }

  ParallelAction getCloseSide15Auto2Gate(
      ScoringSystem scoringSystem, MecanumDrive drive, Telemetry telemetry) {
    double secondShootX = -5.5;
    double secondShootY = -17;
    final int angleOffset;
    final double xOffset = -1;
    if (allianceColor == AllianceColor.RED) {

      angleOffset = -3;
    } else {
      angleOffset = 0;
    }
    return new ParallelAction( // BIGGEST BOI
        new updateScoring(scoringSystem, telemetry),
        // new updateTelemetry(telemetry),
        new SequentialAction( // BIG BOI
            new SequentialAction(new setScoringPose(scoringSystem, allianceColor)), // 1
            // new ObeliskSearch(limelight, telemetry),
            new setAiming(90, -49.5 + angleOffset / -3, 2400, 37, flipy, scoringSystem),
            new SequentialAction( // shoot 1
                drive
                    .actionBuilder(allianceColor.getAutoStartPosition())
                    // -----SHOOT1------\\
                    .splineToLinearHeading(
                        allianceColor.getAutoRRShootPosition(),
                        (Math.PI / -8) * flipy,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 80))
                    .build(),
                new ShootAction(scoringSystem)),
            new setAiming(90, -42 + angleOffset, 2500, 38, flipy, scoringSystem),
            new SequentialAction( // pickup 1 PGP
                drive
                    .actionBuilder(allianceColor.getAutoRRShootPosition())

                    // pickup first (second slot)
                    .splineToLinearHeading(
                        new Pose2d(14, -27.9 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -2) * flipy,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 200))
                    .splineToLinearHeading(
                        new Pose2d(14.25, -59 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -2) * flipy,
                        new TranslationalVelConstraint(180.0),
                        new ProfileAccelConstraint(-30, 140))
                    .waitSeconds(0.05)
                    // shoot 2
                    .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem)),
            new SequentialAction( // pickup2 (gate pickup)
                // GATE PICKUP
                drive
                    .actionBuilder(
                        (new Pose2d(
                            secondShootX, secondShootY * flipy, flipy * (Math.toRadians(-90)))))
                    .splineToLinearHeading(
                        new Pose2d(15, -30 * flipy, flipy * Math.toRadians(-90)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .splineToLinearHeading(
                        new Pose2d(13, -58.5 * flipy, flipy * Math.toRadians(-120)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .waitSeconds(.15)
                    .strafeToLinearHeading(
                        new Vector2d(13.25, -55 * flipy), flipy * Math.toRadians(-120))
                    .strafeToLinearHeading(
                        new Pose2d(16 + xOffset * flipy, -62.5 * flipy, flipy * Math.toRadians(-90))
                            .position,
                        flipy * Math.toRadians(-90),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    // GATE PICKUP
                    .waitSeconds(.59)
                    // shoot3
                    .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem)),
            new SequentialAction(
                // GATE PICKUP
                drive
                    .actionBuilder(
                        (new Pose2d(
                            secondShootX, secondShootY * flipy, flipy * (Math.toRadians(-90)))))
                    .splineToLinearHeading(
                        new Pose2d(15, -30 * flipy, flipy * Math.toRadians(-90)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .splineToLinearHeading(
                        new Pose2d(13, -58.5 * flipy, flipy * Math.toRadians(-120)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .waitSeconds(.15)
                    .strafeToLinearHeading(
                        new Vector2d(13.25, -55 * flipy), flipy * Math.toRadians(-120))
                    .strafeToLinearHeading(
                        new Pose2d(16 + xOffset * flipy, -62.5 * flipy, flipy * Math.toRadians(-90))
                            .position,
                        flipy * Math.toRadians(-90),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    // GATE PICKUP
                    .waitSeconds(.59)
                    // shoot3
                    .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new SetShootingMode(scoringSystem, Spindex.ShootingMode.SLOW),
                new ShootAction(scoringSystem),
                new setAiming(70, -24 + angleOffset, 2455, 39, flipy, scoringSystem)),
            new SequentialAction( // pickup close
                drive
                    .actionBuilder(
                        (new Pose2d(
                            secondShootX, secondShootY * flipy, flipy * (Math.toRadians(-90)))))
                    // go to pickup 4
                    .splineToLinearHeading(
                        // pickup PPG first slot
                        new Pose2d(
                            -16 + xOffset * flipy * 2, -51 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -3) * flipy,
                        new TranslationalVelConstraint(150.0),
                        new ProfileAccelConstraint(-50, 180))
                    .waitSeconds(.1)
                    // shoot 5 and leave3
                    .strafeToLinearHeading(
                        new Pose2d(
                                secondShootX - 30,
                                secondShootY * flipy,
                                flipy * (Math.toRadians(-90)))
                            .position,
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(300.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem)),
            new SetShootingMode(scoringSystem, Spindex.ShootingMode.FAST),
            new AutoEndShutdowAction(scoringSystem)));
  }

  ParallelAction getCloseSide15Auto2GatePickupWith3GateHit(
      ScoringSystem scoringSystem, MecanumDrive drive, Telemetry telemetry) {
    double secondShootX = -5.5;
    double secondShootY = -17;
    final int angleOffset;
    final double xOffset = -1;
    if (allianceColor == AllianceColor.RED) {

      angleOffset = -3;
    } else {
      angleOffset = 0;
    }
    return new ParallelAction( // BIGGEST BOI
        new updateScoring(scoringSystem, telemetry),
        // new updateTelemetry(telemetry),
        new SequentialAction( // BIG BOI
            new SequentialAction(new setScoringPose(scoringSystem, allianceColor)), // 1
            // new ObeliskSearch(limelight, telemetry),
            new setAiming(90, -50.5 + angleOffset / -3, 2400, 24, flipy, scoringSystem),
            new SequentialAction( // shoot 1
                drive
                    .actionBuilder(allianceColor.getAutoStartPosition())
                    // -----SHOOT1------\\
                    .splineToLinearHeading(
                        allianceColor.getAutoRRShootPosition(),
                        (Math.PI / -8) * flipy,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 80))
                    .build(),
                new ShootAction(scoringSystem)),
            new setAiming(90, -43 + angleOffset, 2520, 26, flipy, scoringSystem),
            new SequentialAction( // pickup 1 PGP
                drive
                    .actionBuilder(allianceColor.getAutoRRShootPosition())

                    // pickup first (second slot)
                    .splineToLinearHeading(
                        new Pose2d(14, -27.9 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -2) * flipy,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 200))
                    .splineToLinearHeading(
                        new Pose2d(14.25, -59 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -2) * flipy,
                        new TranslationalVelConstraint(180.0),
                        new ProfileAccelConstraint(-30, 140))
                    .waitSeconds(0.05)
                    .strafeToLinearHeading(
                        new Pose2d(12, -55 * flipy, flipy * (Math.toRadians(-90))).position,
                        flipy * (Math.toRadians(0)))
                    // shoot 2
                    .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem)),
            new SequentialAction( // pickup2 (gate pickup)
                // GATE PICKUP
                drive
                    .actionBuilder(
                        (new Pose2d(
                            secondShootX, secondShootY * flipy, flipy * (Math.toRadians(-90)))))
                    .splineToLinearHeading(
                        new Pose2d(15, -30 * flipy, flipy * Math.toRadians(-90)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .splineToLinearHeading(
                        new Pose2d(13, -59.5 * flipy, flipy * Math.toRadians(-120)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .waitSeconds(.65)
                    .strafeToLinearHeading(
                        new Vector2d(13.25, -55 * flipy), flipy * Math.toRadians(-120))
                    .strafeToLinearHeading(
                        new Pose2d(16 + xOffset * flipy, -62.5 * flipy, flipy * Math.toRadians(-90))
                            .position,
                        flipy * Math.toRadians(-90),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    // GATE PICKUP
                    .waitSeconds(.19)
                    // shoot3
                    .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem)),
            new SequentialAction(
                // GATE PICKUP
                drive
                    .actionBuilder(
                        (new Pose2d(
                            secondShootX, secondShootY * flipy, flipy * (Math.toRadians(-90)))))
                    .splineToLinearHeading(
                        new Pose2d(15, -30 * flipy, flipy * Math.toRadians(-90)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .splineToLinearHeading(
                        new Pose2d(13, -59.5 * flipy, flipy * Math.toRadians(-120)),
                        flipy * Math.PI / -1,
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .waitSeconds(.65)
                    .strafeToLinearHeading(
                        new Vector2d(13.25, -55 * flipy), flipy * Math.toRadians(-120))
                    .strafeToLinearHeading(
                        new Pose2d(16 + xOffset * flipy, -62.5 * flipy, flipy * Math.toRadians(-90))
                            .position,
                        flipy * Math.toRadians(-90),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    // GATE PICKUP
                    .waitSeconds(.19)
                    // shoot3
                    .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(250.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem),
                new setAiming(70, -25 + angleOffset, 2475, 27, flipy, scoringSystem)),
            new SequentialAction( // pickup close
                drive
                    .actionBuilder(
                        (new Pose2d(
                            secondShootX, secondShootY * flipy, flipy * (Math.toRadians(-90)))))
                    // go to pickup 4
                    .splineToLinearHeading(
                        // pickup PPG first slot
                        new Pose2d(
                            -16 + xOffset * flipy * 2, -51 * flipy, flipy * (Math.toRadians(-90))),
                        (Math.PI / -3) * flipy,
                        new TranslationalVelConstraint(150.0),
                        new ProfileAccelConstraint(-50, 180))
                    .waitSeconds(.1)
                    // shoot 5 and leave3
                    .strafeToLinearHeading(
                        new Pose2d(
                                secondShootX - 30,
                                secondShootY * flipy,
                                flipy * (Math.toRadians(-90)))
                            .position,
                        flipy * (Math.toRadians(-90)),
                        new TranslationalVelConstraint(300.0),
                        new ProfileAccelConstraint(-50, 240))
                    .build(),
                new ShootAction(scoringSystem)),
            new AutoEndShutdowAction(scoringSystem)));
  }
}
