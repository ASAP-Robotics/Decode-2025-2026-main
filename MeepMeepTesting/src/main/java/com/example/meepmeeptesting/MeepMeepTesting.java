package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
  // === FILL THESE IN from your AllianceColor methods ===
  // Blue: as-is
  private static final Pose2d START_POSE_BLUE = new Pose2d(0, 0, 0);   // TODO
  private static final Pose2d SHOOT_POSE_BLUE = new Pose2d(0, 0, 0);   // TODO

  // Red: if your code mirrors by flipping Y or uses a different start pose, put the real one here
  private static final Pose2d START_POSE_RED  = new Pose2d(-38.5, 54.6, Math.toRadians(90));   // TODO
  private static final Pose2d SHOOT_POSE_RED  = new Pose2d(-11.3, 24.7, Math.toRadians(90));   // TODO

  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(800);

    RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
            // NOTE: put your real drive constraints here if you want accurate timing.
            // maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth (inches)
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

    // Toggle alliance here
    boolean isRed = true;

    Pose2d beginPose = isRed ? START_POSE_RED : START_POSE_BLUE;
    Pose2d shootPose = isRed ? SHOOT_POSE_RED : SHOOT_POSE_BLUE;

    int flipx = 1;
    int flipy = isRed ? -1 : 1;

    // These match your file
    int pickupAcc = 210;
    int fastAcc   = 240;

    TranslationalVelConstraint vFast = new TranslationalVelConstraint(250.0);
    TranslationalVelConstraint vPick = new TranslationalVelConstraint(250.0);

    ProfileAccelConstraint aFast = new ProfileAccelConstraint(-50, fastAcc);
    ProfileAccelConstraint aPick = new ProfileAccelConstraint(-10, pickupAcc);

    // --- SHOOT 1 path: beginPose -> rrShootPosition
    Action toShoot1 = bot.getDrive().actionBuilder(beginPose)
            .splineToLinearHeading(
                    shootPose,
                    (Math.PI / -8) * flipy,
                    vFast,
                    aFast
            )
            .build();

    // --- PICKUP 1: shootPose -> (14, -27.9) -> (15, -61.5) -> back to shootPose
    Action pickup1AndBack = bot.getDrive().actionBuilder(shootPose)
            .splineToLinearHeading(
                    new Pose2d(14 * flipx, -27.9 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2) * flipy,
                    vFast,
                    aFast
            )
            .splineToLinearHeading(
                    new Pose2d(15 * flipx, -61.5 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2) * flipy,
                    vPick,
                    aPick
            )
            .waitSeconds(0.5)
            .strafeToLinearHeading(
                    shootPose.position,
                    flipy * Math.toRadians(-90),
                    vFast,
                    aFast
            )
            .build();

    // --- PICKUP 2: shootPose -> (13, -35) -> strafe to (13, -62) -> wait -> back to shootPose
    Action pickup2AndBack = bot.getDrive().actionBuilder(shootPose)
            .splineToLinearHeading(
                    new Pose2d(13 * flipx, -35 * flipy, flipy * Math.toRadians(-120)),
                    (Math.PI / -2) * flipy,
                    vFast,
                    aFast
            )
            .strafeTo(new Pose2d(13 * flipx, -62 * flipy, flipy * Math.toRadians(-118)).position)
            .waitSeconds(3.0)
            .strafeToLinearHeading(
                    shootPose.position,
                    flipy * Math.toRadians(-90),
                    vFast,
                    aFast
            )
            .build();

    // --- EXTRA pickup (your code does this as another SequentialAction after pickup2)
    Action extraPickupAndBack = bot.getDrive().actionBuilder(shootPose)
            .splineToLinearHeading(
                    new Pose2d(-11.6 * flipx, -51 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2) * flipy,
                    new TranslationalVelConstraint(240.0),
                    new ProfileAccelConstraint(-50, pickupAcc)
            )
            .waitSeconds(0.5)
            .strafeToLinearHeading(
                    shootPose.position,
                    flipy * Math.toRadians(-90),
                    vFast,
                    aFast
            )
            .build();

    // --- PICKUP 3: shootPose -> (37.1, -25) -> (37.1, -62.5) -> splineTo shootPose
    Action pickup3AndBack = bot.getDrive().actionBuilder(shootPose)
            .splineToLinearHeading(
                    new Pose2d(37.1 * flipx, -25 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2) * flipy,
                    vFast,
                    aFast
            )
            .splineToLinearHeading(
                    new Pose2d(37.1 * flipx, -62.5 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2) * flipy,
                    vFast,
                    new ProfileAccelConstraint(-50, pickupAcc)
            )
            .waitSeconds(0.5)
            .splineTo(
                    shootPose.position,
                    (Math.PI / -2) * flipy,
                    vFast,
                    aFast
            )
            .build();

    // --- LEAVE: shootPose -> (4.2, -43.8)
    Action leave = bot.getDrive().actionBuilder(shootPose)
            .splineToLinearHeading(
                    new Pose2d(4.2 * flipx, -43.8 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2) * flipy,
                    vFast,
                    aFast
            )
            .build();

    // Replace your shooter/mechanism actions with waits so the timeline roughly matches:
    Action routine = new SequentialAction(
            // “ObeliskSearch + aiming” placeholder
            bot.getDrive().actionBuilder(beginPose).waitSeconds(0.2).build(),

            // Drive to shoot 1
            toShoot1,
            // shootAction placeholder
            bot.getDrive().actionBuilder(shootPose).waitSeconds(0.25).build(),

            // pickup 1 + shoot 2
            pickup1AndBack,
            bot.getDrive().actionBuilder(shootPose).waitSeconds(0.25).build(),

            // pickup 2 + shoot 3
            pickup2AndBack,
            bot.getDrive().actionBuilder(shootPose).waitSeconds(0.25).build(),

            // extra pickup + shoot
            extraPickupAndBack,
            bot.getDrive().actionBuilder(shootPose).waitSeconds(0.25).build(),

            // pickup 3 + shoot
            pickup3AndBack,
            bot.getDrive().actionBuilder(shootPose).waitSeconds(0.25).build(),

            // leave + “shutdown” placeholder in parallel (does nothing visual)
            new ParallelAction(
                    leave,
                    bot.getDrive().actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.01).build()
            )
    );

    bot.runAction(routine);

    meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL) // pick whatever
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(bot)
            .start();
  }
}
