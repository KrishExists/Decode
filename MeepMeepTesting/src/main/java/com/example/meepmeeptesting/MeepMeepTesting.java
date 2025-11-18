package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        final Pose2d START_POSE = new Pose2d(-51.5, 51.5, Math.toRadians(-144));
        final Pose2d LSHOOT = new Pose2d(-18, -2.5, Math.toRadians(-225));
        final Pose2d SPIKE1 = new Pose2d(37, 51, Math.toRadians(-270));
        final Pose2d SPIKE2 = new Pose2d(12,    54, Math.toRadians(-270));
        final Pose2d SPIKE3 = new Pose2d(-11.6,     42, Math.toRadians(-270));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(START_POSE)
                .build();

        bot.runAction(
                bot.getDrive()
                        .actionBuilder(START_POSE)

                        // PRELOAD → LSHOOT
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
//
                        // LSHOOT → SPIKE 3
                        .turnTo(Math.toRadians(-230))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(SPIKE3, Math.toRadians(90))
//
                        // SPIKE 3 → Return to LSHOOT
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
//
                        // LSHOOT → SPIKE 2
                        .setTangent(Math.toRadians(20))
                        .splineToSplineHeading(
                                new Pose2d(5, 20, Math.toRadians(-270)),
                                Math.toRadians(-20)
                        )
                        .splineToLinearHeading(SPIKE2, Math.toRadians(-270))

                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)

//
                        // LSHOOT → SPIKE 1
                        .setTangent(Math.toRadians(15))
                        .splineToSplineHeading(
                                new Pose2d(27, 10, Math.toRadians(-270)),
                                Math.toRadians(-15)
                        )
                        .splineToLinearHeading(SPIKE1, Math.toRadians(-270))

                        // SPIKE 1 → LSHOOT
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)

                        // LEAVE
                        .strafeTo(new Vector2d(0, 18))

                        .build()
        );

        meepMeep
                .setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
