package org.firstinspires.ftc.teamcode.roadRunner;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d currentPos = new Pose2d(0,0,Math.toRadians(0));
    public static boolean poseFromAuto = false;
    public static Pose2d currentPose;
}
