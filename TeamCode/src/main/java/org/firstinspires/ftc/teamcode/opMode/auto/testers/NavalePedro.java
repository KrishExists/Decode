
package org.firstinspires.ftc.teamcode.opMode.auto.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class NavalePedro extends OpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState;
    private Paths pathLibrary;
    private Outtake outtake;
    private Intake intake;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        pathLibrary = new Paths(follower);
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathLibrary.path1);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(pathLibrary.path2);
                    pathState = 2;
                    outtake.setPower(0.9);
                    outtake.setLinkage(0.42);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pathLibrary.path3);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pathLibrary.path4);
                    pathState = 4;
                    intake.setPower(1);
                    intake.transfer.setPower(1);

                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pathLibrary.path5);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pathLibrary.path6);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(pathLibrary.path7);
                    pathState = 7;
                }
                break;
            default:
                break;
        }
    }

    public static class Paths {
        public final PathChain path1;
        public final PathChain path2;
        public final PathChain path3;
        public final PathChain path4;
        public final PathChain path5;
        public final PathChain path6;
        public final PathChain path7;

        public Paths(Follower follower) {
            path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.269, 7.615),
                                    new Pose(88.965, 9.494),
                                    new Pose(89.078, 9.799),
                                    new Pose(89.191, 10.105),
                                    new Pose(89.304, 10.410),
                                    new Pose(89.426, 10.739),
                                    new Pose(89.539, 11.044),
                                    new Pose(89.652, 11.349),
                                    new Pose(89.765, 11.654),
                                    new Pose(89.878, 11.960),
                                    new Pose(90.000, 12.288)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                    .build();

            path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.000, 12.288),
                                    new Pose(132.058, 9.038)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.058, 9.038),
                                    new Pose(134.385, 10.058)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(357), Math.toRadians(350))
                    .build();

            path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.385, 10.058),
                                    new Pose(89.923, 12.192)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(70))
                    .setReversed()
                    .build();

            path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.923, 12.192),
                                    new Pose(126.596, 9.404)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.596, 9.404),
                                    new Pose(134.923, 9.192)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.923, 9.192),
                                    new Pose(127.489, 9.794),
                                    new Pose(119.522, 9.294),
                                    new Pose(113.223, 13.454),
                                    new Pose(104.758, 9.440),
                                    new Pose(97.654, 12.215),
                                    new Pose(90.077, 12.269)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(70))
                    .setReversed()
                    .build();
        }

    }
}