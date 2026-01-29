package org.firstinspires.ftc.teamcode.opMode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

@Autonomous(name = "RedAutoFarSIMPLY", group = "Autonomous")
@Configurable
public class RedSimplySynchronization extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Pose startPose = new Pose(80.1, 7.6, Math.toRadians(90));

    private Intake intake;
    private Outtake outtake;
    private DcMotorEx transfer;
    private Servo blocker;

    private ElapsedTime actionTimer;
    private boolean ran = true;
    private boolean happened = false;

    @Override
    public void init() {
        // Initialize Subsystems
        outtake = new Outtake(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, outtake);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        blocker = hardwareMap.get(Servo.class, "blocker");

        actionTimer = new ElapsedTime();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);

        paths = new Paths(follower);

        // Initial Servo/Linkage Positions
        outtake.linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
        blocker.setPosition(0.5);

        panelsTelemetry.debug("Status", "Initialized & Merged");
        panelsTelemetry.update(telemetry);
    }

    // ---------------- Robot Action Helpers (Logic from File A) ----------------

    private void prepareToShoot() {
        intake.setPower(TeamConstants.INTAKE_FEED_POWER);
        outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
        blocker.setPosition(TeamConstants.BLOCKER_OPEN);
    }

    private void spinUpIntake() {
        outtake.spinToRpm(TeamConstants.outtake_Stop);
        intake.setPower(TeamConstants.INTAKE_IN_POWER);
        transfer.setPower(TeamConstants.TRANSFER_IN_POWER_AUTO);
        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
    }

    private void spinUp(boolean withTransfer) {
        outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
        if (withTransfer) {
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

    private void shoot(PathChain nextPath) {
        if (follower.isBusy()) {
            prepareToShoot();
        }

        if (!follower.isBusy()) {
            // Logic: Wait for RPM or bypass if 'happened' is true
            if (outtake.atSpeed(2000, 3000) || happened) {
                happened = true;
                spinUp(true);
                transfer.setPower(-1); // Feed into shooter

                if (actionTimer.milliseconds() > 1000) {
                    follower.followPath(nextPath, true);
                    pathState++;
                    resetBooleans();
                }
            } else {
                spinUp(false);
            }
        }
    }

    private void spinIntake(PathChain path) {
        spinUpIntake();
        if (!follower.isBusy()) {
            follower.followPath(path, true);
            pathState++;
            resetBooleans();
        }
    }

    // ---------------- State Machine ----------------

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to Score Preload
                follower.followPath(paths.score);
                pathState++;
                break;

            case 1: // Shoot Preload, then move to Pick1
                resetTimers();
                shoot(paths.pick1);
                break;

            case 2: // Intaking during move to Pick1, then move to Score
                spinIntake(paths.score);
                break;

            case 3: // Shoot Pick1, then move to PickSimply
                resetTimers();
                shoot(paths.pickSimplyBall);
                break;

            case 4: // Intake SimplyBall, then move to scoreSpecial
                spinIntake(paths.scoreSpecial);
                break;

            case 5: // Shoot scoreSpecial, then move to pickSimply (repeat)
                resetTimers();
                shoot(paths.pickSimplyBall);
                break;

            case 6: // Intake repeat, then move to scoreSpecial
                spinIntake(paths.scoreSpecial);
                break;

            case 7: // Final Shoot, then leave
                resetTimers();
                shoot(paths.leave);
                break;

            case 8: // Done
                if(!follower.isBusy()){
                    outtake.stop();
                    intake.setPower(0);
                }
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("RPM", outtake.getRPM());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain score;
        public PathChain scoreSpecial;
        public PathChain pick1;
        public PathChain pickSimplyBall;
        public PathChain shoot;
        public PathChain leave;

        public Paths(Follower follower) {
            score = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80.182, 7.618),

                                    new Pose(90.894, 10.689)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                    .build();

            pick1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(90.894, 10.689),
                                    new Pose(143.060, 21.944),
                                    new Pose(129.329, 14.744),
                                    new Pose(134.775, 11.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(-60))
                    .addPath(
                            new BezierLine(
                                    new Pose(134.775, 11.218),

                                    new Pose(134.735, 7.863)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-10))
                    .build();



            shoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.735, 7.863),

                                    new Pose(90.672, 10.744)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(65))

                    .build();

            pickSimplyBall = follower.pathBuilder().addPath(//We will do this twice
                            new BezierLine(
                                    new Pose(90.672, 10.744),

                                    new Pose(130.435, 11.035)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-55))
                    .addPath(
                            new BezierLine(
                                    new Pose(130.435, 11.035),

                                    new Pose(136.696, 7.930)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-55), Math.toRadians(0))
                    .build();



            scoreSpecial = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.696, 7.930),

                                    new Pose(90.574, 10.643)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                    .build();


            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(91.130, 10.852),

                                    new Pose(106.730, 14.217)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
                    .build();
        }
    }
}