package org.firstinspires.ftc.teamcode.opMode.auto;
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


@Autonomous(name = "Red Far", group = "Autonomous")
@Configurable // Panels
public class ChowChin extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Pose startPose = new Pose(82, 8.5, Math.toRadians(90));

    private Intake intake;
    private Outtake outtake;
    private DcMotorEx transfer;
    private Servo blocker;

    private ElapsedTime actionTimer;

    private ElapsedTime quickTimerForPath8;
    private boolean ran = true;
    private boolean happened = false;


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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.000, 8.500),

                                    new Pose(85.500, 18.800)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90, 10),

                                    new Pose(100, 30)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(100, 30),

                                    new Pose(140, 30)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(140, 39),

                                    new Pose(85.500, 18.800)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.500, 18.800),

                                    new Pose(136.800, 23.300)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(-90))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.800, 23.300),

                                    new Pose(136.800, 4)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.800, 4),

                                    new Pose(85.500, 18.800)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(65))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.500, 18.800),

                                    new Pose(136, 2)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-0))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136, 2),

                                    new Pose(85.500, 18.800)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(65))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.500, 18.800),

                                    new Pose(85.500, 28.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }


    private void prepareToShoot() {
        intake.setPower(0);
        outtake.spinToRpm(TeamConstants.SHOOTER_FARAUTO_RPM);
        blocker.setPosition(TeamConstants.BLOCKER_OPEN);
        transfer.setPower(0);
    }

    private void stopALL() {
        intake.setPower(0);
        transfer.setPower(0);
        outtake.stop();
    }
    private void spinUpIntake() {
        outtake.spinToRpm(TeamConstants.outtake_Stop);
        intake.setPower(TeamConstants.INTAKE_IN_POWER);
        transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
    }
    private void spinUpShooter() {
        telemetry.addLine("Ready to shoot");
        outtake.spinToRpm(TeamConstants.SHOOTER_FARAUTO_RPM);
    }

    private void spinUp(boolean withTransfer) {
        spinUpShooter();
        if (withTransfer) {
            transfer.setPower(TeamConstants.TRANSFER_IN_POWER_AUTO +0.3);
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
            outtake.spinToRpm(TeamConstants.SHOOTER_FARAUTO_RPM);
            if ((outtake.atSpeed(2950,3200)||happened) ) {
                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
                happened = true;
                spinUp(true);
                transfer.setPower(-1);
                if (actionTimer.milliseconds()>3000 ) {
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

    private void spinIntake(PathChain path,int y) {
        if(follower.isBusy()) {
            spinUpIntake();
        }
        if (!follower.isBusy()) {
            follower.followPath(path,true);
            pathState++;
            resetBooleans();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to Score Preload
                follower.followPath(paths.Path1);
                intake.setPower(1);
                pathState++;
                break;

            case 1: // Shoot Preload, then move to Pick1
                resetTimers();
                shoot(paths.Path2,false);//pick it up
                break;

            case 2: // Intaking during move to Pick1, then move to Score
                spinUpIntake();
                spinIntake(paths.Path3, 110);
                break;

            case 3: // Shoot Pick1, then move to PickSimply
                resetTimers();
                shoot(paths.Path4,false);
                break;

            case 4:
                spinUpIntake();// Intake SimplyBall, then move to scoreSpecial
                spinIntake(paths.Path5, 110);
                break;

            case 5: // Shoot scoreSpecial, then move to pickSimply (repeat)
                spinUpIntake();
                resetTimers();
                follower.setMaxPower(0.8);
                spinIntake(paths.Path6,110);
                break;

            case 6:
                follower.setMaxPower(1);
                resetTimers();// Intake repeat, then move to scoreSpecial
                shoot(paths.Path7, false);
                break;

            case 7: // Final Shoot, then leave
                resetTimers();
                spinIntake(paths.Path8,110);
                break;

            case 8: // Done
                follower.setMaxPower(1);
                resetTimers();
                shoot(paths.Path9, false);
                break;

            case 9:
                stopALL();

        }


    }

    public void init() {
        // Initialize Subsystems
        outtake = new Outtake(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, outtake);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        blocker = hardwareMap.get(Servo.class, "blocker");

        actionTimer = new ElapsedTime();
        quickTimerForPath8 = new ElapsedTime();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);

        paths = new Paths(follower);

        // Initial Servo/Linkage Positions
        outtake.linkage.setPosition(TeamConstants.LINKAGE_REST);
        blocker.setPosition(TeamConstants.BLOCKER_OPEN);

        panelsTelemetry.debug("Status", "Initialized & Merged");
        panelsTelemetry.update(telemetry);
    }


}
    