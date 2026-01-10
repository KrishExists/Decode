package org.firstinspires.ftc.teamcode.opMode.auto.testers; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto", group = "Examples")
public class PedroTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(124.0, 124.0, Math.toRadians(35)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(96.0, 96.0, Math.toRadians(45)); // Scoring Pose of our robot.
    private final Pose Bez1End = new Pose(98.0,84.0,0);
    private final Pose Bez1Control = new Pose(96.0,84.0,0);
    private final Pose Spike1End = new Pose(124.0,84.0,0);
    private final Pose Bez2End = new Pose(98.0,60.0,0);
    private final Pose Bez2Control = new Pose(96.0,60.0,0);
    private final Pose Spike2End = new Pose(124.0,60.0,0);
    private final Pose Bez3End = new Pose(98.0,36.0,0);
    private final Pose Bez3Control = new Pose(96.0,36.0,0);
    private final Pose Spike3End = new Pose(124.0,36.0,0);
    private final Pose scorePoseend = new Pose(90.0, 108.0, Math.toRadians(35)); // Scoring Pose of our robot.

    private Path scorePreload;
    private PathChain
            PrepSpike1, FinishSpike1, ScoreSpike1,
            PrepSpike2, FinishSpike2, ScoreSpike2,
            PrepSpike3, FinishSpike3, ScoreSpike3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        PrepSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, Bez1Control,Bez1End))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Bez1End.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        FinishSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(Bez1End, Spike1End))
                .setTangentHeadingInterpolation()
                .build();
        ScoreSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(Spike1End, scorePose))
                .setLinearHeadingInterpolation(Spike1End.getHeading(),scorePose.getHeading())
                .build();


        PrepSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, Bez2Control, Bez2End))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Bez2End.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        FinishSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(Bez2End, Spike2End))
                .setTangentHeadingInterpolation()
                .build();

        ScoreSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(Spike2End, scorePose))
                .setLinearHeadingInterpolation(Spike2End.getHeading(), scorePose.getHeading())
                .build();
        PrepSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, Bez3Control, Bez3End))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Bez3End.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        FinishSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(Bez3End, Spike3End))
                .setTangentHeadingInterpolation()
                .build();

        ScoreSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(Spike3End, scorePoseend))
                .setLinearHeadingInterpolation(Spike3End.getHeading(), scorePose.getHeading())
                .build();


    }
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(PrepSpike1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(FinishSpike1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(ScoreSpike1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(PrepSpike2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(FinishSpike2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(ScoreSpike2, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(PrepSpike3, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(FinishSpike3, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(ScoreSpike3, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    setPathState(-1); // done
                }
                break;
        }
    }


    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}