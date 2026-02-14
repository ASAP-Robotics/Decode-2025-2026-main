package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
  // === FILL THESE IN from your AllianceColor methods ===
  // Blue: as-is
  private static final Pose2d START_BLUE = new Pose2d(0, 0, 0);   // TODO
  private static final Pose2d SHOOT_BLUE = new Pose2d(0, 0, 0);   // TODO

  // Red: if your code mirrors by flipping Y or uses a different start pose, put the real one here
  private static final Pose2d START_RED  = new Pose2d(-38.5, 54.6, Math.toRadians(90));   // TODO
  private static final Pose2d SHOOT_RED  = new Pose2d(-11.3, 24.7, Math.toRadians(90));   // TODO

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // ===== Pick alliance =====
        boolean isRed = true;

        int flipx = 1;
        int flipy = isRed ? -1 : 1;

        Pose2d startPose = isRed ? START_RED : START_BLUE;
        Pose2d rrShootPose = isRed ? SHOOT_RED : SHOOT_BLUE;

        // from your auto
        int pickupAcc = 210;
        int fastAcc = 240;

        double secondShootX = -5.5;
        double secondShootY = -17;

        TranslationalVelConstraint v250 = new TranslationalVelConstraint(250.0);
        TranslationalVelConstraint v180 = new TranslationalVelConstraint(180.0);
        TranslationalVelConstraint v300 = new TranslationalVelConstraint(300.0);

        ProfileAccelConstraint aFast = new ProfileAccelConstraint(-50, fastAcc);
        ProfileAccelConstraint aPick30 = new ProfileAccelConstraint(-30, pickupAcc);
        ProfileAccelConstraint aPick50 = new ProfileAccelConstraint(-50, pickupAcc);
        ProfileAccelConstraint aFastPlus10 = new ProfileAccelConstraint(-50, fastAcc + 10);

        Pose2d secondShootPose = new Pose2d(
                secondShootX,
                secondShootY * flipy,
                flipy * Math.toRadians(-90)
        );

        // -------------------------
        // “shoot 1” drive segment
        // -------------------------
        Action toShoot1 = bot.getDrive().actionBuilder(startPose)
                .splineToLinearHeading(
                        rrShootPose,
                        (Math.PI / -8) * flipy,
                        v250,
                        aFast
                )
                .splineToLinearHeading(
                        new Pose2d(14 * flipx, -27.9 * flipy, flipy * Math.toRadians(-90)),
                        (Math.PI / -2) * flipy,
                        v250,
                        aFast
                )
                .strafeTo(
                        new Pose2d(15 * flipx, -61.5 * flipy, flipy * Math.toRadians(-90)).position,
                        v250,
                        aPick30
                )
                .waitSeconds(0.2)
                .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * Math.toRadians(-90),
                        v250,
                        aFast
                )
                .build();

        // ---------------------------------------------
        // pickup 1 (second slot) -> strafe to shoot 2
        // ---------------------------------------------

        // -------------------------------------------------------
        // pickup2 (gate pickup) from secondShootPose -> back shoot3
        // -------------------------------------------------------
        Action gatePickup_thenShoot3 = bot.getDrive().actionBuilder(secondShootPose)
                .splineToLinearHeading(
                        new Pose2d(13.5 * flipx, -30 * flipy, flipy * Math.toRadians(-90)),
                        Math.PI * flipy,              // same value as flipy * Math.PI / -1, clearer
                        v250,
                        aFast
                )
                .splineToLinearHeading(
                        new Pose2d(13.5 * flipx, -62 * flipy, flipy * Math.toRadians(-110)),
                        Math.PI * flipy,
                        v250,
                        aFast
                )
                .waitSeconds(1.5)
                .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * Math.toRadians(-90),
                        v250,
                        aFast
                )
                .build();

        // -------------------------------------------------------
        // far pickup (your “pickup 4 gpp”) from secondShootPose
        // then back to secondShoot (shoot4)
        // NOTE: you used splineToLinearHeading without constraints
        // on the second spline in robot code; kept same call style.
        // -------------------------------------------------------
        Action farPickup_thenBackToSecondShoot = bot.getDrive().actionBuilder(secondShootPose)
                .splineToLinearHeading(
                        new Pose2d(43.1 * flipx, -27 * flipy, flipy * Math.toRadians(-90)),
                        (Math.PI / -2) * flipy,
                        v250,
                        aFast
                )
                .splineToLinearHeading(
                        new Pose2d(40.1 * flipx, -62.5 * flipy, flipy * Math.toRadians(-90)),
                        (Math.PI / -2) * flipy
                )
                .strafeToLinearHeading(
                        new Vector2d(secondShootX, secondShootY * flipy),
                        flipy * Math.toRadians(-90),
                        v250,
                        aFast
                )
                .build();

        // -------------------------------------------------------
        // close pickup (PPG first slot) from secondShootPose
        // then strafe to final position (shoot5/leave)
        // -------------------------------------------------------
        Action closePickup_thenFinal = bot.getDrive().actionBuilder(secondShootPose)
                .splineToLinearHeading(
                        new Pose2d(-13.5 * flipx, -49 * flipy, flipy * Math.toRadians(-90)),
                        (Math.PI / -2) * flipy,
                        v180,
                        aPick50
                )
                .waitSeconds(0.9)
                .strafeToLinearHeading(
                        new Pose2d(secondShootX - 30, secondShootY * flipy, flipy * Math.toRadians(-90)).position,
                        flipy * Math.toRadians(-90),
                        v300,
                        aFastPlus10
                )
                .build();

        bot.runAction(new SequentialAction(
                toShoot1,
                gatePickup_thenShoot3,
                farPickup_thenBackToSecondShoot,
                closePickup_thenFinal
        ));

        meepMeep.setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}