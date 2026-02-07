package org.firstinspires.ftc.teamcode.opMode.auto;

import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.opMode.auto.testers.FarAutoPaths;
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
    private FarAutoPaths.Paths paths;
    private Pose startPose = new Pose(79.32891566265059, 7.6, Math.toRadians(90));

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

        paths = new FarAutoPaths.Paths(follower);

        // Initial Servo/Linkage Positions
        outtake.linkage.setPosition(TeamConstants.LINKAGE_REST);
        blocker.setPosition(TeamConstants.BLOCKER_OPEN);

        panelsTelemetry.debug("Status", "Initialized & Merged");
        panelsTelemetry.update(telemetry);
    }

    // ---------------- Robot Action Helpers (Logic from File A) ----------------

    private void prepareToShoot() {
        intake.setPower(0);
        outtake.spinToRpm(TeamConstants.SHOOTER_FAR_RPM);
        blocker.setPosition(TeamConstants.BLOCKER_OPEN);
        transfer.setPower(0);
    }
    private void spinUpIntake() {
        outtake.spinToRpm(TeamConstants.outtake_Stop);
        intake.setPower(TeamConstants.INTAKE_IN_POWER);
        transfer.setPower(TeamConstants.TRANSFER_IN_POWER_AUTO);
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
            outtake.spinToRpm(TeamConstants.SHOOTER_FAR_RPM);
            if ((outtake.atSpeed(2500,3200)||happened) ) {
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
                shoot(paths.scorespike1,false);//pick it up
                break;

            case 2: // Intaking during move to Pick1, then move to Score
                spinIntake(paths.shootspike1);
                break;

            case 3: // Shoot Pick1, then move to PickSimply
                resetTimers();
                shoot(paths.pick1,false);
                break;

            case 4: // Intake SimplyBall, then move to scoreSpecial
                spinIntake(paths.shoot);
                break;

            case 5: // Shoot scoreSpecial, then move to pickSimply (repeat)
                resetTimers();
                shoot(paths.pick1,false);
                break;

            case 6: // Intake repeat, then move to scoreSpecial
                spinIntake(paths.shoot);
                break;

            case 7: // Final Shoot, then leave
                resetTimers();
                shoot(paths.leave,true);
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


}