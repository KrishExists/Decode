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
import org.firstinspires.ftc.teamcode.util.TeamConstants;


@Autonomous
@Config // Panels
public class NavaleOffseasonAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private boolean happened = false;
    private boolean ran = false;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private DcMotorEx transfer;
    private ElapsedTime actionTimer;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(124.006, 118.335, Math.toRadians(38)));

        paths = new Paths(follower);

        turret = new Turret(hardwareMap, telemetry, follower);

        outtake = new Outtake(hardwareMap, telemetry);
        outtake.linkage.setPosition(TeamConstants.LINKAGE_SHOOT); //0.75


        intake = new Intake(hardwareMap, telemetry, outtake, follower);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");

        turret.auto(new Pose(144,144));
        actionTimer = new ElapsedTime();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void start(){
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        turret.auto(new Pose(144,140));
        pathState = autonomousPathUpdate(); // Update autonomous state machine

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
        ran = false;
        happened = false;
    }

    public static class Paths {
        public PathChain scorePreload;
        public PathChain intakeSpike1;
        public PathChain scorePose1;
        public PathChain intakeSpike2;
        public PathChain scorePose2;
        public PathChain goGate;
        public PathChain scorePose3;
        public PathChain intakeSpike3;
        public PathChain scorePoseFINAL;

        public Paths(Follower follower) {
            scorePreload = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(124.006, 118.335),
                                    new Pose(100.089938, 94.6147764)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(330))
                    .build();

            intakeSpike1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.089938, 94.6147764),
                                    new Pose(121.7, 80.1)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            scorePose1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(121.7, 80.1),
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
                                    new Pose(127.6, 60.842)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            scorePose2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(127.6, 60.842),
                                    new Pose(92.3, 83.3)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            goGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.3, 83.3),
                                    new Pose(116.13070438637367,55.95324025078109),
                                    new Pose(129.31255673222392, 59.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(31))
                    .build();

            scorePose3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(129.31255673222392, 59.5),
                                    new Pose(84.398, 72.495)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(31), Math.toRadians(330))
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
        outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
        intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
        transfer.setPower(TeamConstants.TRANSFER_CLOSED);
    }
    private void spinIntake(PathChain nextPath) {
        spinupIntake();
        if (!ran) ran = follower.isBusy();
        if (ran && !follower.isBusy()) {
            follower.followPath(nextPath);
            resetBooleans();
            setPathState(pathState + 1);
        }
    }
    private void shoot(PathChain nextPath, boolean skip) {
        if (!happened) {
            prepareToShoot();
            actionTimer.reset();
        }
        if (!follower.isBusy()) {
//            if(!skip){
//                //turret.auto(new Pose(144,144));
//            }
            if ((outtake.atSpeed(3450,3550)||happened) ) {
                happened = true;
                spinUp(true);
                if (actionTimer.milliseconds()>1200) {
                    if (skip) {
                        setPathState(99); //stops evt
                        return;
                    }
                    follower.followPath(nextPath,true);
                    setPathState(pathState + 1);
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
                spinIntake(paths.scorePose1);
                break;

            case 3: // Shoot at scorePose1, then follow intakeSpike2
                shoot(paths.intakeSpike2);
                break;

            case 4: // Driving to intakeSpike2 — spin up intake along the way
                spinIntake(paths.scorePose2);
                break;

            case 5: // Shoot at scorePose2, then follow goGate
                shoot(paths.goGate);
                break;

            case 6: // Driving to gate — spin up intake along the way
                spinupIntake();
                if (!ran) ran = follower.isBusy(); // wait until we confirm path is running
                if (ran && !follower.isBusy()) {
                    if(!happened){
                        happened = true;
                        actionTimer.reset(); //reseting the timer from the last shoot method
                    }
                    if (actionTimer.milliseconds() > 1500) {
                        follower.followPath(paths.scorePose3);
                        resetBooleans();
                        setPathState(7);
                    }
                }
                break;

            case 7: // Shoot at scorePose3, then follow intakeSpike3
                shoot(paths.intakeSpike3);
                break;

            case 8: // Driving to intakeSpike3 — spin up intake along the way
                spinIntake(paths.scorePoseFINAL);
                break;

            case 9:
                // Final shot, no next path
                turret.auto(new Pose(144,135));
                shoot(null, true);
                telemetry.addLine("Finished");
                break;
            default:
                outtake.setPower(0);
                intake.setPower(0);
                transfer.setPower(0);
                telemetry.addLine("Default");
                break;
        }
        return pathState;
    }
}