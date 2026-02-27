
package org.firstinspires.ftc.teamcode.opMode.auto.testers;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;


@Autonomous(name = "Weird Paths", group = "Testers")
@Configurable // Panels
public class Weirdaths extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Path10;
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.422, 108.940),

                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.000, 84.000),
                                    new Pose(170.000, 82.916),
                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.000, 84.000),
                                    new Pose(50.000, 57.169),
                                    new Pose(210.000, 40.000),
                                    new Pose(118.422, 69.313),
                                    new Pose(102.349, 66.120),
                                    new Pose(87.000, 84.000),
                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.000, 84.000),
                                    new Pose(44.325, 8.855),
                                    new Pose(215.000, 30.000),
                                    new Pose(114.193, 46.024),
                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.000, 84.000),
                                    new Pose(105.301, 48.795),
                                    new Pose(133.952, 61.205)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.952, 61.205),

                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.000, 84.000),
                                    new Pose(105.301, 48.795),
                                    new Pose(133.952, 61.205)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.952, 61.205),

                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.000, 84.000),
                                    new Pose(105.301, 48.795),
                                    new Pose(133.952, 61.205)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.952, 61.205),

                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))

                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1); // start first path
                pathState++;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    pathState++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    pathState++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8);
                    pathState++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    pathState = -1;
                }
                break;


        }
        return pathState;
    }


}
    