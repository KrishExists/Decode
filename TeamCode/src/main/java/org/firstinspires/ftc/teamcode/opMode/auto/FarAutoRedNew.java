package org.firstinspires.ftc.teamcode.opMode.auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FarAutoPahts", group = "Testers")
@Configurable // Panels
public class FarAutoRedNew extends OpMode {
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

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(85.002,10.489 , Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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
        private Pose startPose = new Pose(86, 8);
        private Pose SpikePickControl = new Pose(96.916, 36.491);
        private Pose EndSpikePickup = new Pose(130.731, 36.874);
        private Pose ShootPose = new Pose(85.193, 10.818);
        private Pose EnterHuman = new Pose(136.904, 9.346);






        public Paths(Follower follower) {
            Park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    ShootPose,
                                    new Pose(85.002, 10.489)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            SpikeMarkPickup = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    ShootPose,
                                    SpikePickControl,
                                    EndSpikePickup
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ShootSpike1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    EndSpikePickup,
                                    ShootPose
                            )
                    )
                    .setLinearHeadingInterpolation(0,0)
                    .setReversed()
                    .build();

            HumanPickup = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    ShootPose,
                                    EnterHuman
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            LeaveHumanPlayer = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    EnterHuman,
                                    ShootPose
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            ShootFirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    startPose,
                                    ShootPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0),0)
                    .build();

        }
    }

    private void preparetoshoot(){
        intake.setPower(TeamConstants.INTAKE_STOP);
        outtake.spinToRpm(4500);
        intake.transfer.setPower(TeamConstants.TRANSFER_CLOSED);
    }
    private void intake(){
        intake.setPower(TeamConstants.TRANSFER_IN_POWER);
        intake.transfer.setPower(TeamConstants.TRANSFER_INTAKE_POWER);
    }
    private void spinupeverthing(boolean withTransfer){
        //shooter spinning up
        outtake.spinToRpm(4500);
        outtake.linkage.setPosition(1);
        telemetry.addLine("Ready to Shoot");
        //transfer + intake spin up
        intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
        intake.transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
        intake.setPower(0);
        intake.transfer.setPower(0);
        telemetry.addLine("transfer at 0");
    }
    private void shoot(PathChain nextPath, boolean skip){
        if(follower.isBusy()){
            preparetoshoot();
        }
        if ((outtake.atSpeed(4300,4600)||happened) ) {
            happened = true;
            spinupeverthing(true);
            if (actionTimer.milliseconds()>1200) {
                if (skip) {
                    pathState = 0;
                    return;
                }
                follower.followPath(nextPath,true);
                pathState++;
                resetBooleans();
            }
        } else {
            telemetry.addLine("Outtake not reaching");
            spinupeverthing(false);
            telemetry.update();
        }

    }

    public int autonomousPathUpdate() {
        switch (pathState){
            case 0:
            follower.followPath(paths.ShootFirst);
            pathState++;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(paths.SpikeMarkPickup);
                    pathState++;

                }
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(paths.ShootSpike1);
                    pathState++;

                }
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(paths.HumanPickup);
                    pathState++;

                }
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(paths.LeaveHumanPlayer);
                    pathState++;

                }
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(paths.LeaveHumanPlayer);
                    pathState++;

                }
            case 6:
                if(!follower.isBusy()){
                    follower.followPath(paths.Park);
                    pathState++;

                }
            case 7:

        }
        return pathState;
    }
}