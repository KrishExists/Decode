package org.firstinspires.ftc.teamcode.opMode.auto;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Chinmay Far Paths (Version Final)", group = "Autonomous")
@Configurable // Panels
public class KrishFarRedWithChin extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Timer pathTimer;
    private Intake intake;
    private Outtake outtake;


    private ElapsedTime actionTimer;
    private boolean ran = true;
    private boolean happened = false;

    private Servo blocker;

    private final Pose startPose = new Pose(79.6, 8, Math.toRadians(90));

    private DcMotorEx transfer;

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

        pathState = 0;
        outtake.linkage.setPosition(TeamConstants.LINKAGE_SHOOT);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(79.583, 8.000),

                                    new Pose(90.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.000, 12.000),

                                    new Pose(136.500, 23.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(-90))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.500, 23.500),

                                    new Pose(136.500, 8.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.500, 8.500),

                                    new Pose(90.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(65))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.000, 12.000),

                                    new Pose(112.000, 7.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(112.000, 7.500),

                                    new Pose(134.000, 7.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.000, 7.500),

                                    new Pose(130.000, 7.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130.000, 7.500),
                                    new Pose(132.500, 9.500),
                                    new Pose(136.500, 23.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.500, 23.000),

                                    new Pose(90.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.000, 12.000),

                                    new Pose(112.000, 7.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(112.000, 7.500),

                                    new Pose(134.000, 7.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.000, 7.500),

                                    new Pose(130.000, 7.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130.000, 7.500),
                                    new Pose(132.500, 9.500),
                                    new Pose(136.500, 23.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            Path14 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.500, 23.000),

                                    new Pose(90.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                    .build();

            Path15 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.000, 12.000),

                                    new Pose(105.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(90))

                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1); //GOOD
                pathState++;
                break;
            case 1:
                resetTimers();
                shoot(paths.Path2); //GOOD
                break;
            case 2:
                spinIntake(paths.Path3); // GOOD
                break;
            case 3:
                resetTimers();
                spinIntake(paths.Path4); //GOOD
                break;
            case 4:
                resetTimers();
                shoot(paths.Path5); // GOOD
                break;
            case 5:
                resetTimers();
                spinIntake(paths.Path6); // GOOD
                break;
            case 6:
                resetTimers();
                spinIntake(paths.Path7); // GOOD
                break;
            case 7:
                resetTimers();
                spinIntake(paths.Path8); // GOOD
                break;
            case 8:
                resetTimers();
                spinIntake(paths.Path9); //GOOD
                break;
            case 9:
                resetTimers();
                shoot(paths.Path10); // GOOD
                break;
            case 10:
                resetTimers();
                spinIntake(paths.Path11); // GOOD
                break;

            case 11:
                resetTimers();
                spinIntake(paths.Path12); // GOOD
                break;

            case 12:
                resetTimers();
                spinIntake(paths.Path13); // GOOD
                break;

            case 13:
                resetTimers();
                spinIntake(paths.Path14); //GOOD
                break;

            case 14:
                resetTimers();
                shoot(paths.Path15); //GOOD
                break;

            default:
                outtake.stop();
                intake.setPower(TeamConstants.SHOOTER_CLOSED);
        }
        return pathState;
    }

    private void prepareToShoot() {
        intake.setPower(TeamConstants.INTAKE_FEED_POWER);
        outtake.spinToRpm(TeamConstants.SHOOTER_FAR_RPM);
        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
        transfer.setPower(-1);
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
        outtake.spinToRpm(TeamConstants.SHOOTER_FAR_RPM);
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
                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
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
        if (!follower.isBusy()&&actionTimer.milliseconds()>1500) {
            follower.followPath(path,true);
            pathState++;
            resetBooleans();
        }
    }


}