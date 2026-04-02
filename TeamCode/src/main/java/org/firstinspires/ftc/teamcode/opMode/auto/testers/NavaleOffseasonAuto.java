package org.firstinspires.ftc.teamcode.opMode.auto.testers;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.TeamConstants;


@Autonomous
@Config // Panels
public class NavaleOffseasonAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private boolean happened = false;
    private boolean ran = true;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private DcMotorEx transfer;
    private TeamConstants teamConstants;
    private ElapsedTime actionTimer;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(126.361, 117.693, Math.toRadians(38)));

        paths = new Paths(follower);

        turret = new Turret(hardwareMap, telemetry, follower);

        outtake = new Outtake(hardwareMap, telemetry);
        outtake.linkage.setPosition(0.6);


        intake = new Intake(hardwareMap, telemetry, outtake, follower);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");

        turret.auto(new Pose(144,144));
        actionTimer = new ElapsedTime();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        turret.auto(new Pose(144,144));

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
    public void setPathState(int state) {
        pathState = state;
    }
    private void resetBooleans() {
        ran = true;
        happened = false;
    }

    public static class Paths {
        public PathChain scorePreload;
        public PathChain intakeSpike1;
        public PathChain scorePose1;
        public PathChain intakeSpike2;
        public PathChain scorePose2;
        public PathChain goGate;
        public PathChain intakeGate;
        public PathChain scorePose3;
        public PathChain intakeSpike3;
        public PathChain scorePoseFINAL;

        public Paths(Follower follower) {
            scorePreload = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126.361, 117.693),
                                    new Pose(108.225, 97.184)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(330))
                    .build();

            intakeSpike1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(108.225, 97.184),
                                    new Pose(126.300, 82.121)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            scorePose1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(121.930, 83.690),
                                    new Pose(91.520, 82.656)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(330))
                    .build();

            intakeSpike2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(91.520, 82.656),
                                    new Pose(109.492, 56.062),
                                    new Pose(127.378, 60.842)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            scorePose2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(127.378, 60.842),
                                    new Pose(92.3, 83.3)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            goGate = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(91.785, 83.121),
                                    new Pose(127.6, 61)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(30))
                    .build();

            scorePose3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(127.6, 61),
                                    new Pose(84.398, 72.495)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(330))
                    .build();

            intakeSpike3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(84.398, 72.495),
                                    new Pose(101.310, 33.992),
                                    new Pose(123.347, 34.602)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            scorePoseFINAL = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(123.347, 34.602),
                                    new Pose(91.806, 106)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }
    private void shoot(PathChain nextPath) {
        shoot(nextPath, false);
    }
    //the different robot actions in each path lolz
    private void spinUp(boolean withTransfer) {
        if (withTransfer) {
            transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
            intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
            telemetry.addLine("transfer at 1");

        }else{
            intake.setPower(0);
            transfer.setPower(0);
            telemetry.addLine("transfer at 0");
        }
    }

    public void spinupIntake(){
        intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
        transfer.setPower(TeamConstants.TRANSFER_INTAKE_POWER);

    }
    public void prepareToShoot(){
        outtake.spinToRpm(3150);
    }
    private void shoot(PathChain nextPath, boolean skip) {
        if (!happened) {
            prepareToShoot();
            actionTimer.reset();
        }
        if (!follower.isBusy()) {
            if(!skip){
                turret.auto(new Pose(144,144));
            }
            if ((outtake.atSpeed(3100,3300)||happened) ) {
                happened = true;
                spinUp(true);
                if (actionTimer.milliseconds()>1200) {
                    if (skip) {
                        return;
                    }
                    follower.followPath(nextPath,true);
                    pathState++;
                    resetBooleans();
                }
            } else {
                telemetry.addLine("Outtake not above"); // Code never reaches here
                spinUp(false);
            }
        }

    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Begin driving to score preload
                follower.followPath(paths.scorePreload);
                setPathState(1);
                break;

            case 1: // Shoot preload, then follow intakeSpike1
                shoot(paths.intakeSpike1);
                break;

            case 2: // Driving to intakeSpike1 — spin up intake along the way
                spinupIntake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.scorePose1);
                    resetBooleans();
                    setPathState(3);
                }
                break;

            case 3: // Shoot at scorePose1, then follow intakeSpike2
                shoot(paths.intakeSpike2);
                break;

            case 4: // Driving to intakeSpike2 — spin up intake along the way
                spinupIntake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.scorePose2);
                    resetBooleans();
                    setPathState(5);
                }
                break;

            case 5: // Shoot at scorePose2, then follow goGate
                shoot(paths.goGate);
                break;

            case 6: // Driving to gate — spin up intake along the way
                spinupIntake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.scorePose3);
                    resetBooleans();
                    setPathState(7);
                }
                break;

            case 7: // Shoot at scorePose3, then follow intakeSpike3
                shoot(paths.intakeSpike3);
                break;

            case 8: // Driving to intakeSpike3 — spin up intake along the way
                spinupIntake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.scorePoseFINAL);
                    resetBooleans();
                    setPathState(9);
                }
                break;

            case 9: // Final shot, no next path
                shoot(null, true);
                break;

            case 10: // End state
                outtake.setPower(0);
                intake.setPower(0);
                transfer.setPower(0);
                telemetry.addLine("Finished");
                break;
        }
        return pathState;
    }
}
