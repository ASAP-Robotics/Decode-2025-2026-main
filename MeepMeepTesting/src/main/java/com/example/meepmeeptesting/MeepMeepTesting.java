package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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

    // Matches your AutoRobot flipping logic:
    // default: flipx=1, flipy=1
    // if RED: flipy = -1 (and you also set rotate=Math.PI but never used it in the path)
    int flipx = 1;
    int flipy = isRed ? -1 : 1;

    // === Start / shoot pose placeholders ===
    // Your real code uses:
    //   allianceColor.getAutoStartPosition()
    //   allianceColor.getAutoRRShootPosition()
    //
    // Replace these with your actual numbers from AllianceColor.
    // For now, I’m using reasonable placeholders so the file compiles.
    Pose2d autoStart = new Pose2d(-38.5, -54.6, Math.toRadians(90));
    Pose2d shootPose = new Pose2d(-11.3, -24.7, Math.toRadians(-90));

    myBot.runAction(
        myBot
            .getDrive()
            .actionBuilder(new Pose2d(15 * flipx, -49 * flipy, flipy * Math.toRadians(-90)))

            // shoot 1: start -> shootPose
            /*.splineToLinearHeading(
                    shootPose,
                    (Math.PI / -8),
                    new TranslationalVelConstraint(250.0),
                    new ProfileAccelConstraint(-50, 180)
            )

            // pickup 1
            .splineToLinearHeading(
                    new Pose2d(-11.6 * flipx, -53.7 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2),
                    new TranslationalVelConstraint(175.0),
                    new ProfileAccelConstraint(-10, 110)
            )

            // back and rotate
            .splineToLinearHeading(
                    new Pose2d(-5.5 * flipx, -45.3 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2),
                    new TranslationalVelConstraint(250.0),
                    new ProfileAccelConstraint(-50, 180)
            )

            // hit gate
            .splineToLinearHeading(
                    new Pose2d(-5.5 * flipx, -51.7 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2),
                    new TranslationalVelConstraint(200.0),
                    new ProfileAccelConstraint(-30, 175)
            )
            .waitSeconds(0.5)

            // shoot 2 (your code starts builder at about (-5.4, -53.7); keep close to that)
            .strafeTo(shootPose.position)

            // pickup 2
            .splineToLinearHeading(
                    new Pose2d(14 * flipx, -27.9 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2),
                    new TranslationalVelConstraint(250.0),
                    new ProfileAccelConstraint(-50, 180)
            )

            // pickup 2 also
            .splineToLinearHeading(
                    new Pose2d(15 * flipx, -60.9 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2),
                    new TranslationalVelConstraint(160.0),
                    new ProfileAccelConstraint(-10, 75)
            )

            // pickup 2 also also
            .splineToLinearHeading(
                    new Pose2d(15 * flipx, -49 * flipy, flipy * Math.toRadians(-90)),
                    (Math.PI / -2),
                    new TranslationalVelConstraint(160.0),
                    new ProfileAccelConstraint(-10, 75)
            )*/

            // shoot 3
            .strafeTo(shootPose.position)

            // pickup 3 (two splines in one builder in your code)
            .splineToLinearHeading(
                new Pose2d(37.1 * flipx, -28 * flipy, flipy * Math.toRadians(-90)),
                (Math.PI / -2),
                new TranslationalVelConstraint(250.0),
                new ProfileAccelConstraint(-50, 180))
            .splineToLinearHeading(
                new Pose2d(37.1 * flipx, -59.2 * flipy, flipy * Math.toRadians(-90)),
                (Math.PI / -2),
                new TranslationalVelConstraint(175.0),
                new ProfileAccelConstraint(-10, 110))

            // shoot 4
            .strafeTo(shootPose.position)

            // leave
            .splineToLinearHeading(
                new Pose2d(4.2 * flipx, -43.8 * flipy, flipy * Math.toRadians(-90)),
                (Math.PI / -2),
                new TranslationalVelConstraint(250.0),
                new ProfileAccelConstraint(-50, 180))
            .build());

    meepMeep
        .setBackground(img)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start();
  }
}
