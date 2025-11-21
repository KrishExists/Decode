package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    // -------------------------------------------------
    //  SIMPLE FUNCTIONS YOU CAN CALL
    // -------------------------------------------------
    public static Vector2d pos(double x, double y) {
        return new Vector2d(x, y);
    }

    public static double rad(double deg) {
        return Math.toRadians(deg);
    }

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        // -----------------------------
        //  START POSE → CHANGE THIS
        // -----------------------------
        Pose2d START = new Pose2d(60, 10, rad(-180));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, rad(180), rad(180), 15)
                .setStartPose(START)
                .build();

        bot.runAction(
                bot.getDrive().actionBuilder(START)

                        // ------------------------------------------------
                        //  SUPER SIMPLE COMMANDS — JUST CHANGE NUMBERS
                        // ------------------------------------------------

                        // go(x, y, heading)
                        .strafeToLinearHeading(pos(54, 10), rad(-200))

                        // turn(degrees)
                        .turnTo(rad(70))

                        // go(x, y, heading)
                       //.strafeToLinearHeading(pos(56, 60), rad(-270))
                        //.turnTo(rad(-70))
                        // go(x, y, heading)
                       // .strafeToLinearHeading(pos(54, 20), rad(-200))
                       // .strafeToLinearHeading(pos(54, 10), rad(-200))

                        .strafeToLinearHeading(pos(34, 33), rad(-270))
                        .strafeToLinearHeading(pos(34, 45), rad(-270))
                        .strafeToLinearHeading(pos(54, 10), rad(-200))
                        .strafeToLinearHeading(pos(25,60), rad(0))
                        .strafeToLinearHeading(pos(58, 60), rad(0))
                        .strafeToLinearHeading(pos(54, 10), rad(-200))
                        .strafeToLinearHeading(pos(28, 10), rad(-200))


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
