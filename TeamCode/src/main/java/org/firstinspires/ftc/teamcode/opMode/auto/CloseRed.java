package org.firstinspires.ftc.teamcode.opMode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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

    private final Pose startPose = new Pose(124, 124, Math.toRadians(35));

    // Paths
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

    // ---------------- Path Building ----------------
    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.733, 127.822),

                                new Pose(84.622, 94.667)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.622, 94.667),
                                new Pose(81.700, 82.989),
                                new Pose(125.044, 83.978)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.044, 83.978),

                                new Pose(84.533, 94.578)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.533, 94.578),
                                new Pose(65.300, 55.511),
                                new Pose(125.311, 59.333)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.311, 59.333),

                                new Pose(84.444, 94.756)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.444, 94.756),
                                new Pose(63.989, 31.444),
                                new Pose(125.400, 35.244)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.400, 35.244),

                                new Pose(79.556, 101.756)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))

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
                follower.followPath(Path1);
                outtake.linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
                pathState++;
                break;
            case 1:
                resetTimers();
                shoot(Path2);
                break;
            case 2:
                spinIntake(Path3, true);
                break;
            case 3:
                spinIntake(Path4,false);
                break;
            case 4:
                resetTimers();
                shoot(Path5);
                break;
            case 5:
                spinIntake(Path6, true);
                break;
            case 6:
                spinIntake(Path7, false);
                break;
            case 7:
                pathState = 67;
                break;
            case 8:
                break;
            case 9:
                break;
            case 10:
                resetTimers();
                shoot(Path1, true);
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
