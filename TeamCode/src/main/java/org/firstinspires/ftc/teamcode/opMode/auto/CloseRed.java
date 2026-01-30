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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

@Autonomous(name = "Pedro Pathing Auto Main", group = "Autonomous")
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

    private final Pose startPose = new Pose(121.2, 130, Math.toRadians(36));
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

    // Paths
    private PathChain
            PrepSpike1, ScoreSpike1,
            PrepSpike2, ScoreSpike2,
            GoGate,BackGate,
            PrepSpike3, ScoreSpike3;
    private Servo blocker;

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
                .setLinearHeadingInterpolation(Spike3End.getHeading(), scorePoseEnd.getHeading())
                .build();
    }

    // ---------------- Robot Actions ----------------
    private void prepareToShoot() {
        intake.setPower(TeamConstants.INTAKE_FEED_POWER);
        outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
        blocker.setPosition(TeamConstants.BLOCKER_OPEN);
        transfer.setPower(0.4);
        telemetry.addLine("transfer poewr 0");
    }

    private void spinUpIntake() {
        outtake.spinToRpm(TeamConstants.outtake_Stop);
        intake.setPower(TeamConstants.INTAKE_IN_POWER);
        transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
    }

    private void spinUpShooter() {
        telemetry.addLine("Ready to shoot");
        outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
    }

    private void spinUp(boolean withTransfer) {
        spinUpShooter();
        if (withTransfer) {
            transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
            intake.setPower(TeamConstants.INTAKE_IN_POWER);
            telemetry.addLine("transfer at -1");

        }else{
            intake.setPower(0);
            transfer.setPower(0);
            telemetry.addLine("transfer at 0");
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
            prepareToShoot();
        }
        if (!follower.isBusy()) {
            if ((outtake.atSpeed(2000,3000)||happened) ) {
                happened = true;
                spinUp(true);
                transfer.setPower(-1);
                if (actionTimer.milliseconds()>1000 ) {
                    if (skip) {
                        pathState = 67;
                        return;
                    }
                    follower.followPath(nextPath,true);
                    pathState++;
                    resetBooleans();
                }
            } else {
                telemetry.addLine("OUttake not above"); // Code never reaches here
                spinUp(false);
                transfer.setPower(0);
            }
        }

    }

    private void shoot(PathChain nextPath) {
        shoot(nextPath, false);
    }

    private void spinIntake(PathChain path) {
        spinUpIntake();
        if (!follower.isBusy()) {
            follower.followPath(path,true);
            pathState++;
            resetBooleans();
        }
    }
    private void spinIntakeGate(PathChain path) {
        spinUpIntake();
        if (!follower.isBusy()&&actionTimer.milliseconds()>1000) {
            follower.followPath(path,true);
            pathState++;
            resetBooleans();
        }
    }

    // ---------------- State Machine ----------------
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                pathState++;
                break;
            case 1:
                resetTimers();
                shoot(PrepSpike2);
                break;
            case 2:
                spinIntake(ScoreSpike2);
                break;
            case 3:
                resetTimers();
                shoot(GoGate);
                break;
            case 4:
                resetTimers();
                spinIntakeGate(BackGate);
                break;
            case 5:
                resetTimers();
                shoot(GoGate);
                break;
            case 6:
                resetTimers();
                spinIntakeGate(BackGate);
                break;
            case 7:
                resetTimers();
                shoot(PrepSpike1);
                break;
            case 8:
                spinIntake(ScoreSpike1);
                break;
            case 9:
                resetTimers();
                shoot(PrepSpike3);
                break;
            case 10:
                resetTimers();
                spinIntake(ScoreSpike3);
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
        blocker = hardwareMap.get(Servo.class, "blocker");

        pathTimer = new Timer();
        actionTimer = new ElapsedTime();
        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
        //transfer.setPower(TeamConstants.TRANSFER_IN_POWER);

        buildPaths();
        pathState = 0;
        outtake.linkage.setPosition(TeamConstants.LINKAGE_SHOOT);

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
