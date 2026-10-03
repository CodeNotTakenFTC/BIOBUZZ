package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

public class MeepMeepTesting {

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(650);

        // 1. Define the bot FIRST so 'myBot' exists
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(120), Math.toRadians(120), 30)
                .build();


        // 2. Handle the IOException required by ImageIO.read
        try {
            meepMeep.setBackground(ImageIO.read(new File("MeepMeepTesting/src/main/assets/BIOBUZZ_Field.png")))
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        } catch (IOException e) {
            System.out.println("Could not find the BIOBUZZ image file! Check your path.");
            e.printStackTrace();
        }

        myBot.runAction(
                myBot.getDrive()
                        .actionBuilder(new Pose2d(60, -58, Math.toRadians(180)))
                        .waitSeconds(1.0)
                        //.turnTo(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(60, -59, Math.toRadians(180)), Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(180)), Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(180)), Math.toRadians(90))
                        .waitSeconds(1.0)
                        .splineToLinearHeading(new Pose2d(60, 59, Math.toRadians(180)), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(50, -62, Math.toRadians(180)), Math.toRadians(180))
                        .strafeTo(new Vector2d(20,20))
                        .build()
        );

        meepMeep.start();
    }
}