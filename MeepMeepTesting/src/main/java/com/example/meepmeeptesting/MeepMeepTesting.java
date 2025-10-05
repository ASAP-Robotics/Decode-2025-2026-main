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
                    .setConstraints(120, 63, Math.toRadians(180), Math.toRadians(180), 15)
                    .build();

    myBot.runAction(
            myBot
                    .getDrive()
                    .actionBuilder(new Pose2d(-61.50, -10, Math.toRadians(0)))
                    .waitSeconds(2)
                    //part where we steal for 61 pts
                  //  .splineTo(new Vector2d(-16, 58), Math.toRadians(0))
                  //  .splineTo(new Vector2d(-25, 30), Math.toRadians(0))
                  //  .waitSeconds(1.5)
                    //OPTIONAL ^^^
                    //MORE REALISTIC VARIANT
                    
                    .splineTo(new Vector2d(-16, 58), Math.toRadians(0))
                    .splineTo(new Vector2d(-25, 30), Math.toRadians(0))
                    .waitSeconds(1.5)

                    .splineTo(new Vector2d(-12, -47), Math.toRadians(0))
                    .splineTo(new Vector2d(-16,-10),Math.toRadians(0))
                    .waitSeconds(1.5)
                    .splineTo(new Vector2d(10, -47), Math.toRadians(0))
                    .splineTo(new Vector2d(-16,-10),Math.toRadians(0))
                    .waitSeconds(1.5)
                    .splineTo(new Vector2d(30, -47), Math.toRadians(0))
                  //  .splineTo(new Vector2d(50,-10),Math.toRadians(0)) this is for the optional one
                    .waitSeconds(1.5)
                    .splineTo(new Vector2d(34, -34), Math.toRadians(0))
                    .build());
    meepMeep
            .setBackground(img)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
  }
}
