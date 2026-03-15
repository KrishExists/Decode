package org.firstinspires.ftc.teamcode.opMode.auto;
import android.widget.Spinner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Enter;

@Autonomous(name = "BlueFar", group = "Autonomous")
@Configurable // Panels
public class BlueFar extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private boolean ran = true;
    private boolean happened = false;
    private ElapsedTime actionTimer;
    private boolean skip;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        actionTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-86,8 , Math.toRadians(180)));
        outtake = new Outtake(hardwareMap,telemetry);
        intake = new Intake(hardwareMap,telemetry,outtake,follower);

        paths = new Paths(follower); // Build paths
        turret = new Turret(hardwareMap,telemetry, follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        PoseStorage.pose = follower.getPose();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private void resetBooleans() {
        ran = true;
        happened = false;
    }

    public static class Paths {
        public PathChain Park;
        public PathChain ShootFirst;

        public PathChain SpikeMarkPickup;
        public PathChain ShootSpike1;
        public PathChain HumanPickup;
        public PathChain LeaveHumanPlayer;
        public PathChain Park1;
        private Pose startPose = new Pose(144-86, 8);
        private Pose SpikePickControl = new Pose(144-92, 36.491);
        private Pose EndSpikePickup = new Pose(144-128, 36.874);
        private Pose ShootPose = new Pose(144-85.193, 10.818);
        private Pose EnterHuman = new Pose(144-136.904, 9.346);
        private Pose BackHuman = new Pose(144-125, 8.519, Math.toRadians(180));
        private Pose EnterHuman2 = new Pose(144-130, 9.346);



        public Paths(Follower follower) {
            Park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    ShootPose,
                                    new Pose(144-112.000, 9.346)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                    .build();

            SpikeMarkPickup = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    ShootPose,
                                    SpikePickControl,
                                    EndSpikePickup
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            ShootSpike1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    EndSpikePickup,
                                    ShootPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                    .build();

            HumanPickup = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    ShootPose,
                                    EnterHuman
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    EnterHuman,
                                    BackHuman

                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    BackHuman,
                                    EnterHuman2
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                    .build();

            LeaveHumanPlayer = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    EnterHuman2,
                                    ShootPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                    .build();
            ShootFirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    startPose,
                                    ShootPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                    .build();

        }
    }
    private void prepareToShoot() {
        intake.setPower(TeamConstants.INTAKE_STOP);
        outtake.spinToRpm(4200);
//        blocker.setPosition(TeamConstants.BLOCKER_OPEN);
        intake.transfer.setPower(TeamConstants.TRANSFER_CLOSED);
    }

    private void spinUpIntake() {
        outtake.spinToRpm(TeamConstants.outtake_Stop);
        intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
        intake.transfer.setPower(TeamConstants.TRANSFER_INTAKE_POWER);
    }

    private void spinUpShooter() {
        telemetry.addLine("Ready to shoot");
        outtake.spinToRpm(4200);
    }

    private void spinUp(boolean withTransfer) {
        spinUpShooter();
        if (withTransfer) {
            intake.transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
            intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
            telemetry.addLine("transfer at 1");

        }else{
            intake.setPower(0);
            intake. transfer.setPower(0);
            telemetry.addLine("transfer at 0");
        }
    }

    private void resetTimers() {
        if (!follower.isBusy() && ran) {
            actionTimer.reset();
            ran = false;
        }
    }



    private void shoot(PathChain nextPath, boolean skip) {
        telemetry.addLine("In shoot");
        if (follower.isBusy()) {
            prepareToShoot();
        }
        if (!follower.isBusy()) {
            telemetry.addData("timer",actionTimer.milliseconds());

            turret.blue(new Pose(0,144));

            if ((outtake.atSpeed(4150,4300)) ) {
                happened = true;
                spinUp(true);

            } else {
                telemetry.addLine("Outtake not above"); // Code never reaches here
                spinUp(false);
            }
            if (actionTimer.milliseconds()>3500) {
                if (skip) {
                    pathState = 67;
                    return;
                }
                follower.followPath(nextPath,true);
                pathState++;
                resetBooleans();
            }
        }

    }

    private void shoot(PathChain nextPath) {
        shoot(nextPath, false);
    } // This is the shooting method, shoot then advance to the next path

    private void spinIntake(PathChain path,int y) {
        if(follower.isBusy()) {
            if (follower.getPose().getY() < y) {
                spinUpIntake();
            } else {
                outtake.spinToRpm(0);
                intake.setPower(0);
                intake.transfer.setPower(0);
            }
        }
        if (follower.getCurrentTValue()>0.99) {
            follower.followPath(path,true);
            pathState++;
            resetBooleans();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState){
            case 0:
                follower.followPath(paths.ShootFirst);
                pathState++;
                break;
            case 1:
                resetTimers();
                shoot(paths.SpikeMarkPickup);
                break;
            case 2:
                resetTimers();
                spinIntake(paths.ShootSpike1,250);
                break;
            case 3:
                resetTimers();
                shoot(paths.HumanPickup);
                break;
            case 4:
                resetTimers();
                spinIntake(paths.LeaveHumanPlayer,250);
                break;

            case 5:
                resetTimers();
                shoot(paths.Park);
                break;
            case 6:
                if(!follower.isBusy()){
                    pathState++;
                }
                break;

            default:
                telemetry.addLine("default");
                outtake.setPower(0);
                intake.transfer.setPower(0);
                intake.setPower(0);


        }

        return pathState;
    }
}