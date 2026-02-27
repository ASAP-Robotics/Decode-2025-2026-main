package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.awt.Image;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Image img = null;
        try {
            img = ImageIO.read(new File("MeepMeepTesting/assets/field.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        RoadRunnerBotEntity myBot =
                new DefaultBotBuilder(meepMeep)
                        // maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth
                        .setConstraints(120, 63, Math.toRadians(180), Math.toRadians(180), 15)
                        .build();

        // === Choose alliance ===
        boolean isRed = false; // set true for RED, false for BLUE

        // Your flip logic
        int flipx = 1;              // kept for parity (not used much here)
        int flipy = isRed ? -1 : 1;

        // === Replace these with your real start/shoot poses ===
        Pose2d autoStart = new Pose2d(-38.5, -54.6, Math.toRadians(90));
        Pose2d shootPose = new Pose2d(-11.3, -24.7, Math.toRadians(-90));

        // === Path params from your Action code ===
        double secondShootX = -5.5;
        double secondShootY = -17;

        final double xOffset = 1;
        final double angleOffset = isRed ? -4 : 0; // (not used in drive-only MeepMeep)

        myBot.runAction(
                myBot
                        .getDrive()
                        .actionBuilder(autoStart)

                        // ===== SHOOT 1 DRIVE =====
                        .splineToLinearHeading(
                                shootPose,
                                (Math.PI / -8) * flipy,
                                new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 80))
                        .waitSeconds(0.25) // "shoot"

                        // ===== PICKUP 1 -> SHOOT 2 =====
                        .splineToLinearHeading(
                                new Pose2d(14 + xOffset * flipy, -27.9 * flipy, flipy * Math.toRadians(-90)),
                                (Math.PI / -2) * flipy,
                                new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 200))
                        .splineToLinearHeading(
                                new Pose2d(14.15, -60 * flipy, flipy * Math.toRadians(-90)),
                                (Math.PI / -2) * flipy,
                                new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-30, 200))
                        .waitSeconds(0.15)
                        .strafeToLinearHeading(
                                new Vector2d(secondShootX, secondShootY * flipy),
                                flipy * Math.toRadians(-90),
                                new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 240))
                        .waitSeconds(0.25) // "shoot"

                        // ===== GATE PICKUP -> SHOOT 3 =====
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
                        .waitSeconds(0.65)
                        .strafeToLinearHeading(new Vector2d(13.25, -55 * flipy), flipy * Math.toRadians(-120))
                        .strafeToLinearHeading(
                                new Vector2d(16 + xOffset * flipy, -62.5 * flipy),
                                flipy * Math.toRadians(-90),
                                new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 240))
                        .waitSeconds(0.19)
                        .strafeToLinearHeading(
                                new Vector2d(secondShootX, secondShootY * flipy),
                                flipy * Math.toRadians(-90),
                                new TranslationalVelConstraint(250.0),
                                new ProfileAccelConstraint(-50, 240))
                        .waitSeconds(0.25) // "shoot"

                        // ===== PICKUP CLOSE -> SHOOT 5 =====
                        .splineToLinearHeading(
                                new Pose2d(-16 + xOffset * flipy, -51 * flipy, flipy * Math.toRadians(-90)),
                                (Math.PI / -3) * flipy,
                                new TranslationalVelConstraint(180.0),
                                new ProfileAccelConstraint(-50, 240))
                        .waitSeconds(0.6)
                        .strafeToLinearHeading(
                                new Vector2d(secondShootX - 30, secondShootY * flipy),
                                flipy * Math.toRadians(-90),
                                new TranslationalVelConstraint(300.0),
                                new ProfileAccelConstraint(-50, 200))
                        .waitSeconds(0.25) // "shoot"

                        .build());

        meepMeep
                .setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}