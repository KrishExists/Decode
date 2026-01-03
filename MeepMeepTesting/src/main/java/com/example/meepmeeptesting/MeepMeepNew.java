package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepNew {
    public static void main(String[] args) {
        // Initialize MeepMeep instance
        MeepMeep meepMeep = new MeepMeep(800);

        // Define the positions (updated to match your autonomous code)
        Pose2d START_POSE = new Pose2d(-49.2, 50.1, Math.toRadians(144));
        Pose2d LSHOOT = new Pose2d(-7, -3, Math.toRadians(-221));
        Pose2d SPIKE3 = new Pose2d(-9, -50, Math.toRadians(-270));
        Pose2d Spike1Init = new Pose2d(40, -10, Math.toRadians(-270));
        Pose2d SPIKE2 = new Pose2d(17, -56, Math.toRadians(-275));
        Pose2d SPIKE1 = new Pose2d(40, -56, Math.toRadians(-270));
        Pose2d OpenGate = new Pose2d(14, -35, Math.toRadians(-270));
        Pose2d TouchGate = new Pose2d(14, -57, Math.toRadians(-245));
        Pose2d LEAVE = new Pose2d(9, -3, Math.toRadians(-227));

        // Create the bot with the same specifications
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

        // Define the actions/paths based on your autonomous logic
        bot.runAction(bot.getDrive().actionBuilder(START_POSE)
                .splineToLinearHeading(LSHOOT, LSHOOT.heading)  // PRELOAD → LSHOOT
                .splineToLinearHeading(SPIKE2, Math.toRadians(-270))  // LSHOOT → SPIKE 2
                .strafeToLinearHeading(SPIKE2.component1(), SPIKE2.heading)  // SPIKE 2 → LSHOOT
                .splineToSplineHeading(OpenGate, OpenGate.heading)  // LSHOOT → OPENGATE
                .strafeToLinearHeading(TouchGate.component1(), TouchGate.heading)  // OPENGATE → LSHOOT
                .splineToLinearHeading(SPIKE3, Math.toRadians(-270))  // LSHOOT → SPIKE 3
                .strafeToLinearHeading(LSHOOT.component1(), LSHOOT.heading)  // SPIKE 3 → LSHOOT
                .splineToLinearHeading(Spike1Init, Math.toRadians(-270))  // LSHOOT → SPIKE 1
                .strafeToLinearHeading(SPIKE1.component1(), Math.toRadians(-270))  // SPIKE 1 → LSHOOT
                .strafeToLinearHeading(LEAVE.component1(), LEAVE.heading)  // LSHOOT → LEAVE
                .build()
        );

        // Set up MeepMeep field settings and start the simulation
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
