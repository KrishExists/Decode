package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
//inspired by Gyrobotic Droids

public class ShooterConstants {

    public static Pose GOAL_POS_RED = new Pose(138,138);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = 38; //inches
    public static double SCORE_ANGLE = Math.toRadians(45);
    public static double PASS_THROUGH_THIS_PROB_RADIUS = 5;



}
