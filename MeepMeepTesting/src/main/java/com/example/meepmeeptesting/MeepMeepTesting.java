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

         Pose2d START_POSE = new Pose2d(-49.2, 50.1, Math.toRadians(-144));
         Pose2d LSHOOT = new Pose2d(-5, 3, Math.toRadians(-227));
         Pose2d SPIKE3 = new Pose2d(-8, 50, Math.toRadians(-270));
         Pose2d Spike1Init = new Pose2d(40, 30,Math.toRadians(-270));
         Pose2d SPIKE2 = new Pose2d(14, 56, Math.toRadians(-270));
         Pose2d Spike2SplineSpot = new Pose2d(14, 40, Math.toRadians(-270));
         Pose2d SPIKE1 = new Pose2d(40, 56, Math.toRadians(-270));
         Pose2d OpenGate = new Pose2d(7, 35, Math.toRadians(-270));
         Pose2d TouchGate = new Pose2d(7, 54, Math.toRadians(-255));
         Pose2d ReleaseGate = new Pose2d(10, 53, Math.toRadians(-255));
         Pose2d LEAVE = new Pose2d(9, 3, Math.toRadians(-227));


        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 17)
                .setConstraints(
                        70,     // maxVel
                        70,     // maxAccel
                        Math.toRadians(180), // maxAngVel
                        Math.toRadians(180), // maxAngAccel
                        15      // track width
                )
                .build();

        bot.runAction(
                bot.getDrive().actionBuilder(START_POSE)

                        // PRELOAD → LSHOOT
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
                        // LSHOOT -> Spike2 Spline
                        .splineToSplineHeading(
                            new Pose2d(14, 20, Math.toRadians(-270)),
                            Math.toRadians(-270)
                )
                        //Spike2 Spline -> SPIKE 2
                        .strafeToLinearHeading(SPIKE2.position, SPIKE2.heading)

                        // SPIKE2 -> LSHOOT
                        .strafeToLinearHeading(Spike2SplineSpot.position, Spike2SplineSpot.heading)

                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
                        // LSHOOT -> Gate
                        .splineToSplineHeading(OpenGate, OpenGate.heading)
                        .strafeToLinearHeading(TouchGate.position, TouchGate.heading
                        )

                        // Gate -> LSHOOT
                        .strafeToLinearHeading(ReleaseGate.position, ReleaseGate.heading)
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)

                        //LShoot -> Gate
                        .splineToSplineHeading(OpenGate, OpenGate.heading)
                        .strafeToLinearHeading(TouchGate.position, TouchGate.heading
                        )

                        // Gate -> LSHOOT
                        .strafeToLinearHeading(ReleaseGate.position, ReleaseGate.heading)
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)

                        //LSHOOT -> SPIKE3
                        .strafeToLinearHeading(SPIKE3.position, SPIKE3.heading)

                        //SPIKE3 -> LSHOOT
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)

                        //LSHOOT -> SPIKE1
                        .splineToSplineHeading(Spike1Init, Math.toRadians(-270))
                        .strafeToLinearHeading(SPIKE1.position, Math.toRadians(-270))

                        //SPIKE1 -> LSHOOT
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)

                        // LSHOOT -> LEAVE
                        .strafeToLinearHeading(LEAVE.position, LEAVE.heading)

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
