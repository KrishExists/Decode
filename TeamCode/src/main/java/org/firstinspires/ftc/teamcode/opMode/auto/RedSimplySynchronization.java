
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


@Autonomous(name = "redsimply", group = "Autonomous")
@Configurable // Panels
public class RedSimplySynchronization extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(80.182, 7.618, Math.toRadians(90)));

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
                                    new Pose(80.182, 7.618),

                                    new Pose(90.894, 10.689)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(90.894, 10.689),
                                    new Pose(143.060, 21.944),
                                    new Pose(129.329, 14.744),
                                    new Pose(134.775, 11.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(-60))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.775, 11.218),

                                    new Pose(134.735, 7.863)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-10))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.735, 7.863),

                                    new Pose(90.672, 10.744)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(65))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.672, 10.744),

                                    new Pose(130.435, 11.035)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-55))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.435, 11.035),

                                    new Pose(136.696, 7.930)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-55), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.696, 7.930),

                                    new Pose(90.574, 10.643)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.574, 10.643),

                                    new Pose(130.435, 11.061)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-55))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.435, 11.061),

                                    new Pose(136.696, 8.017)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-55), Math.toRadians(0))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.696, 8.017),

                                    new Pose(91.130, 10.852)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(91.130, 10.852),

                                    new Pose(106.730, 14.217)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch(pathState){
            case 0:
                follower.followPath(paths.Path1);
                pathState++;
                break;

            case 1:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path2);
                    pathState++;
                }
                break;

            case 2:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path3);
                    pathState++;
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path4);
                    pathState++;
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path5);
                    pathState++;
                }
                break;

            case 5:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path6);
                    pathState++;
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path7);
                    pathState++;
                }
                break;

            case 7:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path8);
                    pathState++;
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path9);
                    pathState++;
                }
                break;

            case 9:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path10);
                    pathState++;
                }
                break;

            case 10:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path11);
                    pathState++;
                }
                break;

            case 11:
                break;



        }

        return pathState;
    }


}
    