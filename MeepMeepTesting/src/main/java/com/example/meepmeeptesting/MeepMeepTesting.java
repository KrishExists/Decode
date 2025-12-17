package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    // Helpers
    public static Vector2d pos(double x, double y) { return new Vector2d(x, y); }
    public static double rad(double deg) { return Math.toRadians(deg); }

    @SuppressWarnings("ALL")
    public static void main(String[] args) {

        MeepMeep mm = new MeepMeep(800);

        // === POSES (FROM BLUECLOSE) ===
        Pose2d START  = new Pose2d(-51.5, -51.5, rad(144));
        Pose2d LSHOOT = new Pose2d(-18, -18, rad(225));
        Pose2d SPIKE1 = new Pose2d(36, -51, rad(270));
        Pose2d SPIKE2 = new Pose2d(12, -51, rad(270));
        Pose2d SPIKE3 = new Pose2d(-12, -51, rad(270));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(mm)
                .setConstraints(
                        60,     // maxVel
                        60,     // maxAccel
                        rad(180),
                        rad(180),
                        15
                )
                .setStartPose(START)
                .build();

        bot.runAction(
                bot.getDrive().actionBuilder(START)

                        // ===== PRELOAD =====
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)

                        // ===== CYCLE 3 =====
                        .turnTo(rad(270))
                        .setTangent(rad(305))
                        .splineToLinearHeading(SPIKE3, rad(270))

                        // ===== SHOOT FROM 3 =====
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)

                        // ===== CYCLE 2 =====
                        .setTangent(rad(340))
                        .splineToSplineHeading(
                                new Pose2d(4, -26, rad(270)),
                                rad(340)
                        )
                        .splineToLinearHeading(SPIKE2, rad(270))

                        // ===== SHOOT FROM 2 =====
                        .setTangent(rad(180))
                        .splineToLinearHeading(
                                new Pose2d(LSHOOT.position, LSHOOT.heading),
                                rad(180)
                        )

                        // ===== CYCLE 1 =====
                        .setTangent(rad(345))
                        .splineToSplineHeading(
                                new Pose2d(27, -27, rad(270)),
                                rad(345)
                        )
                        .splineToLinearHeading(SPIKE1, rad(270))

                        // ===== SHOOT FROM 1 =====
                        .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)

                        // ===== LEAVE =====
                        .strafeTo(pos(0, -18))

                        .build()
        );

        mm.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
