package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
//cURRENT
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d START_POSE = new Pose2d(60, 0  , Math.toRadians(180));
        Pose2d LSHOOT = new Pose2d(58, 3, Math.toRadians(144));
       //Pose2d GOSHOOT2 = new Pose2d(-23, 26, Math.toRadians(-235));

        Pose2d SPIKE1 = new Pose2d(59, 60, Math.toRadians(-270));
        Pose2d SPIKE2 = new Pose2d(59, 60, Math.toRadians(-270));
        Pose2d SPIKE3 = new Pose2d(59, 60, Math.toRadians(70));
        Pose2d OpenGate = new Pose2d(0, 35, Math.toRadians(-270));
        Pose2d TouchGate = new Pose2d(0, 54, Math.toRadians(-270));
       // Pose2d DontTouchGate = new Pose2d(14, 42, Math.toRadians(-270))

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
                      //  .turnTo(Math.toRadians(-270))
                       // .setTangent(Math.toRadians(0))
//                        .turnTo(90)
//                        .strafeToLinearHeading(SPIKE3.position, Math.toRadians(90))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(SPIKE3.position.minus(new Vector2d(0,10)), Math.toRadians(70)), Math.toRadians(70))
                        .setTangent(Math.toRadians(70))
                        .splineToSplineHeading(SPIKE3, Math.toRadians(90))

                        // SPIKE 3 → Opening the Gate
                       // .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
                      //  .strafeToLinearHeading(TouchGate.position, TouchGate.heading)

//                        //Opening the Gate -> LSHOOT
//                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
//
//                        // LSHOOT → SPIKE 2
////                        .setTangent(Math.toRadians(20))
////                        .strafeToLinearHeading(
////                                new Pose2d(11, 10,Math.toRadians(-270)).position,
////                                Math.toRadians(-270)
////                        )
//                        .strafeToLinearHeading(SPIKE2.position, Math.toRadians(-270))

                        // SPIKE 2 → LSHOOT
//                        .setTangent(Math.toRadians(-90))
//                        .strafeToLinearHeading(
//                                new Pose2d(11, 20,Math.toRadians(-10)).position,
//                                Math.toRadians(-270)
//                        )
//                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
//
////                        .splineToSplineHeading(
////                                new Pose2d(27, 10, Math.toRadians(-270)),
////                                Math.toRadians(-15)
////                        )
//
//
//                        // LSHOOT → SPIKE 1
////                        .setTangent(Math.toRadians(15))
////                        .splineToSplineHeading(
////                                new Pose2d(27, 10, Math.toRadians(-270)),
////                                Math.toRadians(-15)
////                        )
//                        .strafeToLinearHeading(SPIKE1.position, Math.toRadians(-270))
//
//                        // SPIKE 1 → LSHOOT
//                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
//
//                        // LEAVE
//                        .strafeTo(new Vector2d(48, 18))

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
