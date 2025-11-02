package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Example Auto", group = "Examples")
public class newPaths extends OpMode {

    private Follower follower;

    private int pathState;
    Pose pose1 = new Pose(70.000, 80.000);
    Pose pose2 = new Pose(34.120, 83.277);
    Pose pose3 = new Pose(16.578, 84.434);
    Pose pose4 = new Pose(70.000, 80.000);
    // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Path scorePreload;
    private PathChain Path1, Path2, Path3,start;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(120))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(170))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(67))
                .build();
        start = follower
                .pathBuilder()
                .addPath(new BezierLine(follower::getPose,pose1))
                .setLinearHeadingInterpolation(follower.getHeading(),Math.toRadians(67))
                .build();
// Define paths


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(start);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Path1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Path2, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Path3, true);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(follower.getPose());

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}
