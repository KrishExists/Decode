
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


@Autonomous(name = "FarAutoRedPathsOnly", group = "Examples")
@Configurable // Panels
public class FarAutoPaths extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Pose startPose = new Pose(80.1,7.6,Math.toRadians(90));

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

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
        public PathChain score;
        public PathChain pick1;
        public PathChain shoot;
        public PathChain leave;
        public PathChain scorespike1;
        public PathChain shootspike1;


        public Paths(Follower follower) {
            score = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80.1, 7.6),

                                    new Pose(90, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                    .build();

            pick1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                  new Pose(90,10),new Pose(111,7)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(111, 7),

                                    new Pose(136, 8)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();



            shoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136, 8),

                                    new Pose(90, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            scorespike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90, 10),

                                    new Pose(100, 35)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(100, 35),

                                    new Pose(135, 35)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shootspike1 =  follower.pathBuilder()
                    .addPath(
                            new  BezierLine(
                                    new Pose(135,35),
                                    new Pose(90,10))
                    )

                    .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(65))

            .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90, 10),

                                    new Pose(106.730, 14.217)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch(pathState){
            case 0:
                follower.followPath(paths.score,true);
                pathState++;
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(paths.scorespike1,true);
                    pathState++;
                }
                break;

            case 2:
                if(!follower.isBusy()){
                    follower.followPath(paths.shootspike1,true);
                    pathState++;
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    follower.followPath(paths.pick1,true);
                    pathState++;
                }
                break;

            case 5:
                if(!follower.isBusy()){
                    follower.followPath(paths.shoot,true);
                    pathState++;
                }
                break;

            case 6:
                if(!follower.isBusy()){
                    follower.followPath(paths.pick1,true);
                    pathState++;
                }
                break;

            case 7:
                if(!follower.isBusy()){
                    follower.followPath(paths.shoot,true);
                    pathState++;
                }
                break;

            case 8:
                if(!follower.isBusy()){
                    follower.followPath(paths.leave);
                    pathState++;
                }
                break;


        }

        return pathState;
    }


}
    