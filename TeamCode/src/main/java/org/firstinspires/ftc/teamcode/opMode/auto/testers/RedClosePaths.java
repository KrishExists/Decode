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
@Autonomous(name = "Paths Red (Full Auto Paths)", group = "Examples")
public class RedClosePaths extends OpMode {

    private Follower follower;
    private int pathState;

    // ---- POSES FROM FULL AUTO ----
    private final Pose startPose = new Pose(123.2612669937254, 116.44743671567421, Math.toRadians(90));
    private final Pose scorePose = new Pose(84, 84, Math.toRadians(0));
    private final Pose scorePoseEnd = new Pose(90, 105, Math.toRadians(0));

    private final Pose Bez1Control = new Pose(85, 55, 0);
    private final Pose Spike1End = new Pose(125, 84, 0);

    private final Pose Bez2Control = new Pose(85, 60, 0);
    private final Pose Spike2End = new Pose(135, 60, 0);

    private final Pose Bez3Control = new Pose(86, 27, 0);
    private final Pose Spike3End = new Pose(135, 36, 0);

    private final Pose Gate = new Pose(132.78313253012047, 60.4578313253012, Math.toRadians(35));
    private final Pose GateControl = new Pose(118.04819277108433, 60.4578313253012, 0);
    private final Pose GateSpec = new Pose(134,58, Math.toRadians(85));
    private final Pose backGate = new Pose(96, 67, 0);

    private Path scorePreload;
    private PathChain PrepSpike1, ScoreSpike1,
            PrepSpike2, ScoreSpike2,
            GoGate, BackGate,
            PrepSpike3, ScoreSpike3;

    private void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        PrepSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, Spike1End))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Spike1End.getHeading())
                .build();

        ScoreSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(Spike1End, scorePose))
                .setLinearHeadingInterpolation(Spike1End.getHeading(), scorePose.getHeading())
                .build();

        GoGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, GateControl, Gate))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Gate.getHeading())
                .addPath(new BezierLine(Gate, GateSpec))
                .setLinearHeadingInterpolation(Gate.getHeading(), GateSpec.getHeading())
                .build();

        BackGate = follower.pathBuilder()
                .addPath(new BezierCurve(GateSpec, backGate, scorePose))
                .setLinearHeadingInterpolation(GateSpec.getHeading(), scorePose.getHeading())
                .build();

        PrepSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, Bez2Control, Spike2End))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Spike2End.getHeading())
                .build();

        ScoreSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(Spike2End, Bez2Control, scorePose))
                .setLinearHeadingInterpolation(Spike2End.getHeading(), scorePose.getHeading())
                .build();

        PrepSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, Bez3Control, Spike3End))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Spike3End.getHeading())
                .build();

        ScoreSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(Spike3End, scorePoseEnd))
                .setLinearHeadingInterpolation(Spike3End.getHeading(), scorePoseEnd.getHeading())
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                pathState++;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(PrepSpike2, true);
                    pathState++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(ScoreSpike2, true);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(GoGate, true);
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(BackGate, true);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(PrepSpike1, true);
                    pathState++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(ScoreSpike1, true);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(PrepSpike3, true);
                    pathState++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(ScoreSpike3, true);
                    pathState = -1;
                }
                break;
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
