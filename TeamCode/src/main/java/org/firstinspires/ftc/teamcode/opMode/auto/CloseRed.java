package org.firstinspires.ftc.teamcode.opMode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

@Autonomous(name = "Pedro Pathing Auto (Integrated)", group = "Autonomous")
public class CloseRed extends OpMode {

    private Follower follower;
    private TelemetryManager panelsTelemetry;

    private int pathState;
    private Timer pathTimer;
    private Intake intake;
    private Outtake outtake;
    private ElapsedTime actionTimer;
    private boolean ran = true;
    private boolean happened = false;

    private DcMotorEx transfer;

    private final Pose startPose = new Pose(125, 125, Math.toRadians(35));
    private final Pose scorePose = new Pose(96, 96, Math.toRadians(45));
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

    // Paths
    private PathChain
            PrepSpike1, FinishSpike1, ScoreSpike1,
            PrepSpike2, FinishSpike2, ScoreSpike2,
            GoGate,BackGate,
            PrepSpike3, FinishSpike3, ScoreSpike3;

    // ---------------- Path Building ----------------
    public void buildPaths() {
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
    }

    // ---------------- Robot Actions ----------------
    private void prepareToShoot() {
        // intake.setPower(TeamConstants.INTAKE_FEED_POWER); // Good
        //  transfer.setPower(TeamConstants.TRANSFER_REV); // Good
        outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM-50); // Good
    }

    private void spinUpIntake() {
        outtake.spinToRpm(TeamConstants.outtake_Stop);
        intake.setPower(TeamConstants.INTAKE_IN_POWER);
        transfer.setPower(TeamConstants.TRANSFER_CLOSED);
    }

    private void spinUpShooter() {
        telemetry.addLine("Ready to shoot");
        outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM-150);
        outtake.setLinkage(TeamConstants.LINKAGE_SHOOT);
    }

    private void spinUp(boolean withTransfer) {
        spinUpShooter();
        if (withTransfer) {
            telemetry.addLine("It is at the transfer state where it is about to shoot");
            transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
            intake.setPower(TeamConstants.INTAKE_IN_POWER);
        }
    }

    private void resetTimers() {
        if (!follower.isBusy() && ran) {
            actionTimer.reset();
            ran = false;
        }
    }

    private void resetBooleans() {
        ran = true;
        happened = false;
    }

    private void shoot(PathChain nextPath, boolean skip) {
        if (follower.isBusy()) {
            prepareToShoot(); // It comes here
            telemetry.addLine("Preparing");
        }

        if (!follower.isBusy()) {
            if ((true) ) { // Good
                telemetry.addLine("Outtake above threshold"); // Good
                outtake.setLinkage(TeamConstants.LINKAGE_SHOOT);
                spinUp(true);
                transfer.setPower(-1);

                if (actionTimer.milliseconds()>1000 ) {
                    if (skip) {
                        pathState = 67;
                        return;
                    }
                    follower.followPath(nextPath);
                    pathState++;
                    resetBooleans();
                }
            } else {
                telemetry.addLine("OUttake not above"); // Code never reaches here
                spinUp(false);
            }
        }
    }

    private void shoot(PathChain nextPath) {
        shoot(nextPath, false);
    }

    private void spinIntake(PathChain path, boolean b) {
        spinUpIntake();
        if (!follower.isBusy()) {

            follower.followPath(path,b);
            pathState++;
            resetBooleans();
        }
    }

    // ---------------- State Machine ----------------
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                outtake.linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
                pathState++;
                break;
            case 1:
                resetTimers();
                shoot(PrepSpike2);
                break;
            case 2:
                spinIntake(ScoreSpike2, false);
                break;
            case 3:
                resetTimers();
                shoot(GoGate);
                break;
            case 4:
                spinIntake(BackGate,false);
                break;
            case 5:
                resetTimers();
                shoot(GoGate);
                break;
            case 6:
                spinIntake(BackGate, false);
                break;
            case 7:
                resetTimers();
                shoot(PrepSpike1);
                break;
            case 8:
                spinIntake(ScoreSpike1, false);
                break;
            case 9:
                resetTimers();
                shoot(PrepSpike3);
                break;
            case 10:
                resetTimers();
                shoot(ScoreSpike3,true);
                pathState = 67;
                break;

            default:
                outtake.stop();
                intake.setPower(TeamConstants.SHOOTER_CLOSED);
        }
    }

    // ---------------- OpMode Methods ----------------
    @Override
    public void init() {
        outtake = new Outtake(hardwareMap,telemetry);
        intake = new Intake(hardwareMap,telemetry,outtake);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        outtake.linkage.setPosition(TeamConstants.LINKAGE_SHOOT);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);

        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        pathTimer = new Timer();
        actionTimer = new ElapsedTime();

        buildPaths();
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        PoseStorage.pose = follower.getPose();

        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("Shooter rpm",outtake.getRPM());
        panelsTelemetry.debug("Transfer",transfer.getVelocity());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
    }
}
