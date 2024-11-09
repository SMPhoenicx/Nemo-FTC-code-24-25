package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose=new Pose2d(-33,-62,Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(23, -52, Math.toRadians(90)))
                .splineTo(new Vector2d(0,-35),Math.toRadians(90))
                //wait ant put speciamskdfjas;dlfkja onto the cage
                .waitSeconds(2)
                        .lineToYLinearHeading(-38,Math.toRadians(-90))
                .splineTo(new Vector2d(48,-38),Math.toRadians(90))
//                        .splineTo(new Vector2d(0,-35),Math.toRadians(-90))
//                .splineTo(new Vector2d(Vector2d37.5,-25),Math.toRadians(0))
                //do intake
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}