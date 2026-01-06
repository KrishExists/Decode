package org.firstinspires.ftc.teamcode.opMode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;
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
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

@Autonomous(name = "Pedro Pathing Auto (Integrated)", group = "Autonomous")
public class test extends OpMode {

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

    private final Pose startPose = new Pose(124, 124, Math.toRadians(45));

    // Paths
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

    // ---------------- Path Building ----------------
    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(124, 124), new Pose(96, 96)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(96, 96), new Pose(96, 84), new Pose(104, 84)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(104, 84), new Pose(124, 84)))
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(124, 84), new Pose(96, 96)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(96, 96), new Pose(96, 60), new Pose(105, 60)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(105, 60), new Pose(124, 60)))
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(124, 60), new Pose(96, 96)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Path8 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(96, 96), new Pose(96, 36), new Pose(105, 36)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(105, 36), new Pose(124, 36)))
                .setTangentHeadingInterpolation()
                .build();

        Path10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(124, 36), new Pose(87, 110)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }

    // ---------------- Robot Actions ----------------
    private void prepareToShoot() {
        intake.setPower(TeamConstants.INTAKE_FEED_POWER);
        transfer.setPower(TeamConstants.TRANSFER_REV);
        outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
    }

    private void spinUpIntake() {
        outtake.spinToRpm(TeamConstants.outtake_Stop);
        intake.setPower(TeamConstants.INTAKE_IN_POWER);
        transfer.setPower(TeamConstants.TRANSFER_CLOSED);
    }

    private void spinUpShooter() {
        outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
        outtake.setLinkage(TeamConstants.LINKAGE_SHOOT);
    }

    private void spinUp(boolean withTransfer) {
        spinUpShooter();
        if (withTransfer) {
            transfer.setPower(TeamConstants.TRANSFER_REV);
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
        if (follower.isBusy()) prepareToShoot();

        if (!follower.isBusy()) {
            if (outtake.getRPM() > TeamConstants.Shooter_BottomThreshold || happened) {
                happened = true;
                spinUp(true);

                if (actionTimer.milliseconds() > 800) {
                    if (skip) {
                        pathState = 67;
                        return;
                    }
                    follower.followPath(nextPath);
                    pathState++;
                    resetBooleans();
                }
            } else {
                spinUp(false);
            }
        }
    }

    private void shoot(PathChain nextPath) {
        shoot(nextPath, false);
    }

    private void spinIntake(PathChain path) {
        spinUpIntake();
        if (!follower.isBusy()) {
            follower.followPath(path);
            pathState++;
            resetBooleans();
        }
    }

    // ---------------- State Machine ----------------
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1);
                pathState++;
                break;
            case 1:
                resetTimers();
                shoot(Path2);
                break;
            case 2:
                spinIntake(Path3);
                break;
            case 3:
                spinIntake(Path4);
                break;
            case 4:
                resetTimers();
                shoot(Path5);
                break;
            case 5:
                spinIntake(Path6);
                break;
            case 6:
                spinIntake(Path7);
                break;
            case 7:
                resetTimers();
                shoot(Path8);
                break;
            case 8:
                spinIntake(Path9);
                break;
            case 9:
                spinIntake(Path10);
                break;
            case 10:
                resetTimers();
                shoot(Path1, true);
                break;
        }
    }

    // ---------------- OpMode Methods ----------------
    @Override
    public void init() {
        outtake = new Outtake(hardwareMap,telemetry);
        intake = new Intake(hardwareMap,telemetry,outtake);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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

        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
    }
}
