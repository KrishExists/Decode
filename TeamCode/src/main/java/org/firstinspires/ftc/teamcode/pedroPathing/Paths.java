package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;

    public Paths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(70.000, 80.000), new Pose(34.120, 83.277))
                )
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(120))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(34.120, 83.277), new Pose(16.578, 84.434))
                )
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(170))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.578, 84.434), new Pose(70.000, 80.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(67))
                .build();
    }
}

