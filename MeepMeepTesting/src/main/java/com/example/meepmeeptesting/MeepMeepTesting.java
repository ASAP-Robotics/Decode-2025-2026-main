package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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
    MeepMeep meepMeep = new MeepMeep(700);
    Image img = null;
    try {
      img = ImageIO.read(new File("MeepMeepTesting/assets/field.png"));
    } catch (IOException e) {
      e.printStackTrace();
    }

    RoadRunnerBotEntity myBot =
            new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(100, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .build();
//CRAZY AHH AUTO [blue]
    myBot.runAction(
            myBot
                    .getDrive()
                    .actionBuilder(new Pose2d(-38, -53, Math.toRadians(0)))
                    .waitSeconds(3)
                    .splineTo(new Vector2d(-12, -47), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-40, -47), Math.toRadians(0))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(10, -47), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-40, -47), Math.toRadians(0))
                    .waitSeconds(1)
                    //^ chunk 1 48 points if sorted.
                    .splineTo(new Vector2d(1,-47), Math.toRadians(0))
                    .strafeTo(new Vector2d(1, -55))
                    .waitSeconds(4)
                    .strafeTo(new Vector2d(1, -53))
                    .splineTo(new Vector2d(62, -25), Math.toRadians(0))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(62, -55), Math.toRadians(270))
                    .waitSeconds(0.5)
                    .splineToConstantHeading(new Vector2d(62, -28), Math.toRadians(270))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(62, -55), Math.toRadians(270))
                    .waitSeconds(0.5)
                    .splineToConstantHeading(new Vector2d(62, -28), Math.toRadians(270))
                    .waitSeconds(1)
                    .build());

//CRAZY AHH AUTO [red]
/*    myBot.runAction(
            myBot
                    .getDrive()
                    .actionBuilder(new Pose2d(-38, 53, Math.toRadians(0)))
                    .waitSeconds(3)
                    .splineTo(new Vector2d(-12, 47), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-40, 47), Math.toRadians(0))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(10, 47), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-40, 47), Math.toRadians(0))
                    .waitSeconds(1)
                    //^ chunk 1 48 points if sorted.
                    .splineTo(new Vector2d(1,47), Math.toRadians(0))
                    .strafeTo(new Vector2d(1, 55))
                    .waitSeconds(4)
                    .strafeTo(new Vector2d(1, 53))
                    .splineTo(new Vector2d(62, 25), Math.toRadians(0))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(62, 55), Math.toRadians(90))
                    .waitSeconds(0.5)
                    .splineToConstantHeading(new Vector2d(62, 28), Math.toRadians(90))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(62, 55), Math.toRadians(90))
                    .waitSeconds(0.5)
                    .splineToConstantHeading(new Vector2d(62, 28), Math.toRadians(90))
                    .waitSeconds(1)
                    .build()); */



    meepMeep
            .setBackground(img)
            .setDarkMode(false)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
  }
}
