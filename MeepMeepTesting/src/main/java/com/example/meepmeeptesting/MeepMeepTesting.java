package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {

    private static Vector2d blueBucket = new Vector2d(58, 56);
    private static Vector2d firstYellow = new Vector2d(48, 28);
    private static Vector2d secondYellow = new Vector2d(58, 28);
    private static Vector2d thirdYellow = new Vector2d(60, 28);
    private static Vector2d parkAscentIntermediate = new Vector2d(40, 10);
    private static Vector2d parkAscent = new Vector2d(24, 10);
    private static Vector2d parkObservation = new Vector2d(-52,61);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(66.741946, 671.80824008, Math.toRadians(351.028), Math.toRadians(268.90909090), 12.4)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(7.5, 61, Math.toRadians(180)))
                .setReversed(true)
                .splineTo(blueBucket, Math.toRadians(45))

                .waitSeconds(0.5)

                .setReversed(false)
                .splineTo(firstYellow, Math.toRadians(-90))

                .waitSeconds(0.1)

                .setReversed(true)
                .splineTo(blueBucket, Math.toRadians(45))

                .waitSeconds(0.5)

                .setReversed(false)
                .splineTo(secondYellow, Math.toRadians(-90))

                .waitSeconds(0.1)

                .setReversed(true)
                .splineTo(blueBucket, Math.toRadians(45))

                .waitSeconds(0.5)

                .setReversed(false)
                .splineTo(thirdYellow, Math.toRadians(0))

                .waitSeconds(0.1)

                .setReversed(true)
                .splineTo(blueBucket, Math.toRadians(45))

                .waitSeconds(0.5)

                // PARK IN ASCENT
                .setReversed(false)
                .splineTo(parkAscentIntermediate, Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(parkAscent, Math.toRadians(0)), Math.toRadians(135))

//                  // PARK IN OBSERVATION
//                .setReversed(false)
//                .splineTo(parkObservation, Math.toRadians(180))

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}