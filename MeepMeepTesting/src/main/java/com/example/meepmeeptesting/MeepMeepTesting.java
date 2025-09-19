package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.Vector2d;

import com.acmerobotics.roadrunner.Pose2d;
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
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(100, 50, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

    myBot.runAction(
        myBot
            .getDrive()
            .actionBuilder(new Pose2d(61.5, 30, Math.toRadians(180)))
                .splineTo(new Vector2d(-20,-20),Math.toRadians(220))
                .waitSeconds(3)
                .splineTo(new Vector2d(48,47),Math.toRadians(180))
                .splineTo(new Vector2d(40,47),Math.toRadians(180)) // i want this to be slower
                .splineTo(new Vector2d(-20,-20),Math.toRadians(220))
                .waitSeconds(3)

                .splineTo(new Vector2d(20,47),Math.toRadians(180))
                .splineTo(new Vector2d(12,47),Math.toRadians(180))
                .splineTo(new Vector2d(-20,-20),Math.toRadians(220))
                .waitSeconds(3)
               /* .splineTo(new Vector2d(0,47),Math.toRadians(180))
                .splineTo(new Vector2d(-8,47),Math.toRadians(180))
                .splineTo(new Vector2d(-20,-20),Math.toRadians(220))
                .waitSeconds(3)*/
                .build());
    /*myBot.runAction(
            myBot
                    .getDrive()
                    .actionBuilder(new Pose2d(58, 26, Math.toRadians(210)))
                    .waitSeconds(3)
                    .splineTo(new Vector2d(48,47),Math.toRadians(180))
                    .splineTo(new Vector2d(40,47),Math.toRadians(180)) // i want this to be slower
                    .splineTo(new Vector2d(58, 26), Math.toRadians(210))
                    .waitSeconds(3)
                    .splineTo(new Vector2d(20,47),Math.toRadians(180))
                    .splineTo(new Vector2d(12,47),Math.toRadians(180))
                    .splineTo(new Vector2d(58, 26), Math.toRadians(210))
                    .waitSeconds(3)
                    .splineTo(new Vector2d(0,47),Math.toRadians(180))
                    .splineTo(new Vector2d(-8,47),Math.toRadians(180))
                    .splineTo(new Vector2d(58, 26), Math.toRadians(210))
                    .waitSeconds(3)
                    .build()); */
    meepMeep.setBackground(img)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();

  }
}
