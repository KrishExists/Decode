package org.firstinspires.ftc.teamcode.opMode.auto.testers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BasicTests", group = "Examples")
public class BaicTests extends OpMode {

    private Follower follower;
    private int pathState;

    private final Pose startPose = new Pose(125, 125, Math.toRadians(35));
    private final Pose scorePose = new Pose(84, 84, Math.toRadians(45));
    private final Pose scorePoseEnd = new Pose(90, 115, Math.toRadians(20));

    private final Pose Bez1End = new Pose(98, 84, 0);
    private final Pose Bez1Control = new Pose(85, 84, 0);
    private final Pose Spike1End = new Pose(120, 84, 0);

    private final Pose Bez2End = new Pose(98, 60, 0);
    private final Pose Bez2Control = new Pose(85, 60, 0);
    private final Pose Spike2End = new Pose(120, 60, 0);

    private final Pose Bez3End = new Pose(98, 36, 0);
    private final Pose Bez3Control = new Pose(78, 36, 0);
    private final Pose Spike3End = new Pose(120, 36, 0);
    private final Pose Gate = new Pose(127,62,Math.toRadians(20));
    private final Pose GateControl = new Pose(98,69,0);
    private final Pose backGate = new Pose(96,67,0);
    private Path scorePreload;
    private PathChain PrepSpike1, ScoreSpike1, PrepSpike2, ScoreSpike2, GoGate, BackGate, PrepSpike3, ScoreSpike3,test;

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        PrepSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, Bez1Control, Bez1End))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Bez1End.getHeading())
                .addPath(new BezierLine(Bez1End, Spike1End))
                .setTangentHeadingInterpolation()
                .build();

        ScoreSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(Spike1End, scorePose))
                .setLinearHeadingInterpolation(Spike1End.getHeading(), scorePose.getHeading())
                .build();
        GoGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,GateControl,Gate))
                .setLinearHeadingInterpolation(scorePose.getHeading(),Gate.getHeading())
                .build();
        BackGate = follower.pathBuilder()
                .addPath(new BezierCurve(Gate,backGate,scorePose))
                .setLinearHeadingInterpolation(Gate.getHeading(),scorePose.getHeading())
                .build();

        PrepSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, Bez2Control, Bez2End))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Bez2End.getHeading())
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
                .addPath(new BezierLine(Bez3End, Spike3End))
                .setTangentHeadingInterpolation()
                .build();

        ScoreSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(Spike3End, scorePoseEnd))
                .setLinearHeadingInterpolation(Spike3End.getHeading(), scorePose.getHeading())
                .build();
        test = follower.pathBuilder()
                .addPath(new BezierLine(startPose, new Pose(72,72)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), 0)
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(test);
                setPathState(5);
                break;

        }
    }

    private void setPathState(int pState) {
        pathState = pState;
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }
}
