package org.firstinspires.ftc.teamcode.opMode.auto.pathsAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class SimplySyncPaths {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    public SimplySyncPaths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(80.178, 7.644),

                                new Pose(81.600, 17.867)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(81.600, 17.867),
                                new Pose(103.978, 22.178),
                                new Pose(97.489, 6.778),
                                new Pose(135.733, 9.067)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(135.733, 9.067),

                                new Pose(81.244, 18.489)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(81.244, 18.489),

                                new Pose(136.022, 23.978)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(136.022, 23.978),

                                new Pose(81.800, 18.467)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(81.800, 18.467),
                                new Pose(79.200, 52.378),
                                new Pose(137.400, 46.911)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(137.400, 46.911),
                                new Pose(82.833, 53.889),
                                new Pose(81.156, 18.200)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                .build();
    }
}