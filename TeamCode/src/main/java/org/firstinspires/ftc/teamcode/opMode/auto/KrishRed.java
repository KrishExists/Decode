package org.firstinspires.ftc.teamcode.opMode.auto;
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


@Autonomous(name = "Krish Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class KrishRed extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123.91111111111113, 128.17777777777778, Math.toRadians(35)));

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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.911, 128.178),

                                    new Pose(77.333, 85.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(35))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(77.333, 85.600),
                                    new Pose(78.811, 58.011),
                                    new Pose(128.778, 59.178)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.778, 59.178),

                                    new Pose(77.267, 85.667)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(77.267, 85.667),
                                    new Pose(116.067, 51.333),
                                    new Pose(133.178, 62.111)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(15))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.178, 62.111),

                                    new Pose(77.311, 85.800)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(35))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(77.311, 85.800),
                                    new Pose(116.511, 52.300),
                                    new Pose(133.444, 62.133)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(15))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.444, 62.133),

                                    new Pose(76.933, 85.756)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(25))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(76.933, 85.756),

                                    new Pose(125.533, 83.333)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.533, 83.333),

                                    new Pose(77.067, 85.778)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(77.067, 85.778),
                                    new Pose(82.222, 30.900),
                                    new Pose(125.556, 35.133)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.556, 35.133),

                                    new Pose(76.422, 92.311)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))

                    .build();
        }
    }


    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path1, true);
                    pathState++;
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    pathState++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    pathState++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    pathState++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    pathState++;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, true);
                    pathState++;
                }
                break;

            case 10:
                // Done — hold pose, do nothing
                break;
        }

        return pathState;
    }



}
    