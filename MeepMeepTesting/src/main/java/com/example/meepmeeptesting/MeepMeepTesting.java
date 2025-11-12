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
    MeepMeep meepMeep = new MeepMeep(670);
    Image img = null;
    try {
      img = ImageIO.read(new File("MeepMeepTesting/assets/field.png"));
    } catch (IOException e) {
      e.printStackTrace();
    }

    RoadRunnerBotEntity myBot =
            new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(150, 75, Math.toRadians(180), Math.toRadians(180), 15)
                    .build();


    myBot.runAction(
            myBot
                    .getDrive()
                    .actionBuilder(new Pose2d(-38, -53, Math.toRadians(0)))
                    .waitSeconds(3)
                    .splineTo(new Vector2d(-12, -47), Math.toRadians(45))
                    .splineToConstantHeading(new Vector2d(-40, -47), Math.toRadians(0))
                    .waitSeconds(1)
                    //there back shoot
                    .splineTo(new Vector2d(10, -47), Math.toRadians(-45))
                    .splineToConstantHeading(new Vector2d(11, -48), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-40, -47), Math.toRadians(0))
                    .waitSeconds(1)
                    //there back shoot
                    //^ chunk 1 48 points if sorted.
                    .splineTo(new Vector2d(1,-47), Math.toRadians(0))
                    .strafeTo(new Vector2d(1, -55))
                    .waitSeconds(3)
                    //hit gate
                    .strafeTo(new Vector2d(1, -53))
                    .splineToConstantHeading(new Vector2d(25, -50), Math.toRadians(0))
                    .splineTo(new Vector2d(34, -46), Math.toRadians(0))
                    //leave and shoot
                    .splineTo(new Vector2d(34,-46),Math.toRadians(-90))
                    .splineTo(new Vector2d(34,-46),Math.toRadians(-180))
                    .splineTo(new Vector2d(34,-46),Math.toRadians(270))
                    .splineTo(new Vector2d(59, -22), Math.toRadians(270))
                    .strafeTo(new Vector2d(62, -22))
                    .waitSeconds(1)
                    //load shoot
                    .splineTo(new Vector2d(62, -49), Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(62, -22), Math.toRadians(270))
                    .waitSeconds(1)
                    //load shoot
                    .splineTo(new Vector2d(62, -53), Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(62, -22), Math.toRadians(270))
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(50,-22))
                    .build());
//CRAZY AHH AUTO [blue]
 /*   myBot.runAction(
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
                    .build()); */

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
