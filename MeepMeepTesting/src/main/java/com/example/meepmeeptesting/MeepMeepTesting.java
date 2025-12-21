package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d START_POSE = new Pose2d(-49.2, 50.1  , Math.toRadians(-144));
        Pose2d LSHOOT = new Pose2d(-22, 3, Math.toRadians(-235));
        Pose2d GOSHOOT2 = new Pose2d(-23, 26, Math.toRadians(-235));

        Pose2d SPIKE1 = new Pose2d(36, 51, Math.toRadians(-270));
        Pose2d SPIKE2 = new Pose2d(14, 54, Math.toRadians(-270));
        Pose2d SPIKE3 = new Pose2d(-10, 52, Math.toRadians(-270));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 17)
                .setConstraints(
                        60,     // maxVel
                        60,     // maxAccel
                        Math.toRadians(180), // maxAngVel
                        Math.toRadians(180), // maxAngAccel
                        15      // track width
                )
                .build();

        bot.runAction(
                bot.getDrive().actionBuilder(START_POSE)

                        // PRELOAD → LSHOOT
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)



                        // GOSHOOT2 → SPIKE 3
                        .turnTo(Math.toRadians(-255))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(SPIKE3, Math.toRadians(90))

                        // SPIKE 3 → LSHOOT
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)

                        // LSHOOT → SPIKE 2
                        .setTangent(Math.toRadians(20))
                        .splineToSplineHeading(
                                new Pose2d(5, 20, Math.toRadians(-270)),
                                Math.toRadians(-20)
                        )
                        .splineToLinearHeading(SPIKE2, Math.toRadians(-270))

                        // SPIKE 2 → LSHOOT
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)

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

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
